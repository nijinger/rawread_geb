#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <fcntl.h>

#include "global.h"
#include "HFC.h"

#define SQR(x)         ((x)*(x))

typedef struct CHICOEVENT_struct
{
  unsigned long long int  LEDts;
  unsigned short int      cathode_tdc_num;
  int                     cathode_tdc_val[128];
  unsigned short int      cathode_tdc_ch[128];
  unsigned short int      anode_tdc_num;
  int                     anode_tdc_val[128];
  unsigned short int      anode_tdc_ch[128];
  unsigned short int      anode_qdc_num;
  int                     anode_qdc_val[32];
  unsigned short int      anode_qdc_ch[32];
  unsigned int            status;
  int			  RF;
  bool			  SINGLE;
} CHICOEVENT;

typedef struct PARTICLE_struct
{
  int   id;
  double t;
  int    dT;  //left - right
  float  dL;
  float  dR;
  int  thetaL;
  int  phiL;
  int  thetaR;
  int  phiR;
  float  fthetaL;
  float  fphiL;
  float  fthetaR;
  float  fphiR;
  int    eL;
  int    eR;
  double rf;
  bool single;
  float Mass;
  bool back;
} PARTICLE;

using namespace std;

void ReadParticle(PARTICLE &particle,BYTE* EventBuf,int length){
  unsigned char *pos=EventBuf;
  int cnt=0;
  memcpy(&(particle.id),pos,4);
  pos+=4;
  cnt+=4;
  memcpy(&(particle.t),pos,56);
  pos+=56;
  cnt+=56;
  memcpy(&(particle.eR),pos,4);
  pos+=4;
  cnt+=4;
  memcpy(&(particle.rf),pos,8);
  pos+=8;
  cnt+=8;
  memcpy(&(particle.single),pos,4);
  pos+=4;
  cnt+=4;
  memcpy(&(particle.Mass),pos,4);
  pos+=4;
  cnt+=4;
  memcpy(&(particle.back),pos,4);
  cnt+=4;
  if((cnt!=length)){
    cout<<"cnt : "<<cnt<<endl;
    cout<<"lengthe: "<<length<<endl;
  }
  assert(cnt==length);
}

struct Mode3event
{
  int length;
  int board_id;
  int chn_id;
  int module;
  long long LED_ts;
  int en;
  bool en_sign;
  int pileup;
  long long CFD_ts;
  int CFD_1;
  int CFD_2;
  int trace_len;
  INT16 trace[8192];
};

void swapbytes(char* a, char *b)
{
  char tmp=*a;
  *a=*b;
  *b=tmp;
}

// Mode 3 data is high endian format
void HEtoLE(char* cBuf, int bytes) {
  for(int i=0; i<bytes; i+=2) 
    swapbytes((cBuf+i), (cBuf+ i+1));
} 

void Mode3Event(char* cBuf, int len, Mode3event* mode3) {
  // Welcome in the 16 bit world
  UINT16* wBuf= (UINT16*)cBuf;
  
  // digitizer saves in high endian format
  // we're low endian
  HEtoLE(cBuf, len);
  
  // length now in units of 16bit words
  len/=2;
  
  // 1st & 2nd word are 0xaaaa
  if((*wBuf != 0xaaaa) && (*(wBuf+1) != 0xaaaa)) {
    cerr << "0xAAAA header missing" << endl;
    return;
  }
  wBuf+=2;

  // 3rd word
  // Digitizer reports length in 32bit units
  // we convert in 16bit. Furthermore the length
  // doesn't account for the two 0xAAAA words
  
  mode3->length = (*wBuf & 0x07ff) * 2 + 2;
  if(mode3->length != len) {
    cerr << "inconsistent mode3 buffer length "
	 << "Geb Hdr: " << len << " wrds "
	 << "Mode2: " << mode3->length << " wrds"
	 <<endl;
    return;
  }
  
  // also board id encoded (=GA ?)
  mode3->board_id = *wBuf >> 11;
  wBuf++;
  
  // 4th word
  mode3->chn_id = *wBuf & 0x000f;
  mode3->module = *wBuf >> 4;
  wBuf++;

  // 5th, 6th and 8th word LED timestamp
  // 5th 'middle', 6th 'lowest', 8th 'highest'

  mode3->LED_ts = ((long long) *(wBuf+3)) << 32;
  mode3->LED_ts += ((long long) *(wBuf+0)) << 16;
  mode3->LED_ts += ((long long) *(wBuf+1)) << 0 ;
  wBuf+=2; //point 7th
  
  // 7th is low 16bit energy
  // 10th upper 8bit, 9th bit sign
  mode3->en = (int) *(wBuf+3) & 0x00ff;
  mode3->en = mode3->en << 16;
  mode3->en += *wBuf;
  
  mode3->en_sign = *(wBuf+3) & 0x0100;
  mode3->pileup  = (*(wBuf+3) & 0x8000) >> 15;
  wBuf+=2; //point 9th

  // 9th, 11th and 12th word CFD ts
  // 9th 'lower, 11th 'highest', 12th 'middle'
  mode3->CFD_ts = ((long long) *(wBuf+2)) << 32;
  mode3->CFD_ts += ((long long) *(wBuf+3)) << 16;
  mode3->CFD_ts += ((long long) *(wBuf+0)) << 0 ;
  wBuf+=4; //point 13th
  
  // 13th, 14th CFD point 1
  mode3->CFD_1 = (int) *(wBuf+1) << 16;
  mode3->CFD_1 += (int) *wBuf;
  wBuf+=2; //point 15th
  
  // 15th, 16th CFD point 2
  mode3->CFD_2 = (int) *(wBuf+1) << 16;
  mode3->CFD_2 += (int) *wBuf;
  wBuf+=2; //point 17th
  
  // wBuf points at 1st trace element now
  mode3->trace_len = mode3->length - 16; 
  
  for(int i=0; i<mode3->trace_len/2; i++) {
#define OFFSET 512;
    mode3->trace[2*i+1]  =-(INT16)(*(wBuf+1)) + OFFSET;
    mode3->trace[2*i+0]=-(INT16)(*(wBuf+0)) + OFFSET;
    wBuf+=2;
  }
  
  cerr << hex
       << " LED: 0x" << mode3->LED_ts
       << " CFD: 0x" << mode3->CFD_ts
       << dec << endl;
}

void BrowseData(gebData header) {
  switch(header.type){
    case 1:
    	cout<<"type: DECOMP\t";
	break;
    case 2:
    	cout<<"type: RAW\t";
	break;
    case 3:
    	cout<<"type: TRACK\t";
	break;
    case 4:
    	cout<<"type: BGS\t";
	break;
    case 5:
    	cout<<"type: S800_RAW\t";
	break;
    case 6:
    	cout<<"type: NSCLnonevent\t";
	break;
    case 7:
    	cout<<"type: GT_SCALER\t";
	break;
    case 8:
    	cout<<"type: GT_MOD29\t";
	break;
    case 9:
    	cout<<"type: S800PHYSDATA\t";
	break;
    case 11:
    	cout<<"type: G4SIM\t";
	break;
    case 12:
    	cout<<"type: CHICO\t";
	break;
    default:
    	cout<<"type : "<<header.type<<"\t";
	break;
  }
  cout<< " len: " << header.length
      << " ts: 0x" << hex << header.timestamp << dec
      << endl;
}


int HFC_mode3(BYTE* cBuf, HFC* hfc_list) {
  /* Return value: processed data in bytes */

  long long mode3_ts;
  int mode3_len;
 
  // 15th and 16th byte is ts high (and endian)
  mode3_ts = ((long long) cBuf[14]) << 40;
  mode3_ts += ((long long) cBuf[15]) << 32;
  // 9th, 10th ts middle 
  mode3_ts += ((long long) cBuf[8]) << 24;
  mode3_ts += ((long long) cBuf[9]) << 16;
  // 11th, 12th ts 'low'
  mode3_ts += ((long long) cBuf[10]) << 8;
  mode3_ts += ((long long) cBuf[11]);

  // 5th, 6th is length, in 32bit units
  mode3_len = ((int)(cBuf[4])) << 8;
  mode3_len += ((int)cBuf[5]);
  mode3_len &= 0x7ff;
  mode3_len *= 4; // convert into bytes

  hfc_list->add(mode3_ts, 2, mode3_len+4, cBuf);
  
  return (mode3_len + 4); // 0xaaaa 0xaaaa not counted in mode3_len
}

int main(int argc, char** argv) {
  if(argc==1) {
    cerr<<argv[0]<<" #filename"<<endl;
    exit(0);
  }
  string filename = argv[1];
  
  FILE *in = fopen64(filename.c_str(),"rb");
  if(!in){
    cerr<<" Cannot open file "<<filename<<endl;
    return 1;
  }

  gebData aGeb;
  BYTE cBuf[8*16382];
  PARTICLE chico;
  int read;
  int cnt=0;
  char option;
  while(fread(&aGeb,sizeof(struct gebData),1,in)==1){
    cnt++;
    read = fread(cBuf, sizeof(char), aGeb.length,in);
    BrowseData(aGeb);
    if(aGeb.type==12){
      ReadParticle(chico,cBuf,aGeb.length);
      cout<<chico.dL<<"\t"<<chico.phiL<<"\t"<<chico.fphiL<<"\t"<<chico.fthetaL<<"\t"<<chico.thetaL<<"\t"<<chico.thetaR<<" "<<sizeof(struct PARTICLE_struct)<<endl;
    }
    if(cnt%10==0) {
      option=getchar();
      if(!strcmp(&option, "q")){
        break;
      }
    }
  }
}

	 
