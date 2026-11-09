// PSK31 decoder benchmark: repeated "The quick brown fox - TEST TEST" through Psk31Modulator, white noise (SNR in 3 kHz), random start phase.
// Build: g++ -O2 -I../src -o bench_psk bench_psk.cpp     Run: ./bench_psk 40 <tuning offset Hz> [carrier Hz]
// Env: SHOW=1 lists bad decodes; LOSS=0.1 LOSSMODE=1|2|3 drops 10% of 20 ms frames (remove / repeat previous / zeros); PPM=300 sample-clock error.
#ifndef DSPH
#define DSPH "dsp.h"
#endif
#include DSPH
#include <vector>
#include <string>
#include <random>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
using namespace std;
static mt19937 rng(1);
static float gauss(){ static normal_distribution<float> n(0,1); return n(rng); }
static int lev(const string&a,const string&b){ vector<int> p(b.size()+1),c(b.size()+1); for(size_t j=0;j<=b.size();j++)p[j]=j; for(size_t i=1;i<=a.size();i++){c[0]=i; for(size_t j=1;j<=b.size();j++) c[j]=min({p[j]+1,c[j-1]+1,p[j-1]+(a[i-1]==b[j-1]?0:1)}); p=c;} return p[b.size()]; }
int main(int argc,char**argv){
  int N=argc>1?atoi(argv[1]):30; double tuneOff=argc>2?atof(argv[2]):0; double sigHz=argc>3?atof(argv[3]):1000;
  const char* msg="M0WWJ M0WWJ M0WWJ TEST TEST";
  float snrs[]={30,15,10,6,3,0};
  printf("tune offset %.1f Hz: char error rate (%%) / %% of runs with >30%% errors (consistently wrong), per SNR in 3 kHz\n",tuneOff);
  for(float snr:snrs){
    int errs=0,tot=0,bad=0;
    for(int t=0;t<N;t++){
      rng.seed(100+t*31+(int)snr);
      dsp::Psk31Modulator mod; mod.begin(sigHz,0.5f); double drift=getenv("DRIFT")?atof(getenv("DRIFT")):0; 
      string full; for(int r=0;r<3;r++){ full+=msg; full+="\r"; }
      // random idle lead so the symbol clock phase differs per run
      int lead=(int)(uniform_real_distribution<float>(0,1)(rng)*256*20);
      mod.queueText(full.c_str()); mod.finish();
      vector<int16_t> audio; int16_t blk[160];
      double A=16000.0; // modulator peak ~ 32767*0.5
      double sigP=A*A/2.0; double noiseP=sigP/pow(10.0,snr/10.0); double sigma=sqrt(noiseP*8000.0/3000.0 /1.0 * 1.0);
      // noise sigma^2 over 4 kHz Nyquist; SNR in 3 kHz => noise power total = noiseP * (4000/3000)
      sigma=sqrt(noiseP*4000.0/3000.0);
      for(int i=0;i<lead;i++) audio.push_back((int16_t)(sigma*gauss()));
      while(!mod.done()){ mod.generate(blk,160); for(int i=0;i<160;i++){ double v=blk[i]+sigma*gauss(); audio.push_back((int16_t)max(-32000.0,min(32000.0,v))); } }
      for(int i=0;i<8000;i++) audio.push_back((int16_t)(sigma*gauss()));
      if(getenv("PPM")){ double ppm=atof(getenv("PPM"))*1e-6; vector<int16_t> r2; for(double x=0; x<audio.size()-1; x+=1.0+ppm){ size_t k=(size_t)x; double f=x-k; r2.push_back((int16_t)(audio[k]*(1-f)+audio[k+1]*f)); } audio.swap(r2); }
      dsp::Psk31Decoder dec; dec.setFreq(sigHz+tuneOff);
      string out; char c;
      double lossP=getenv("LOSS")?atof(getenv("LOSS")):0; int lossMode=getenv("LOSSMODE")?atoi(getenv("LOSSMODE")):1;
      vector<int16_t> prev(160,0);
      for(size_t i=0;i<audio.size();i+=160){
        size_t m=min((size_t)160,audio.size()-i);
        bool lost = uniform_real_distribution<float>(0,1)(rng) < lossP;
        if(lost){ if(lossMode==2) dec.process(prev.data(),160); else if(lossMode==3) { vector<int16_t> z(160,0); dec.process(z.data(),160);} }
        else { dec.process(&audio[i],m); if(m==160) copy(audio.begin()+i,audio.begin()+i+160,prev.begin()); }
        while(dec.getChar(c)) out+=(c=='\n'?'\r':c);
      }
      string ref=full; 
      auto strip=[](string s){string r; for(char ch:s) if(ch!='\r'&&ch!='\n') r+=ch; return r;};
      string a=strip(ref), b=strip(out);
      int e=lev(a,b); errs+=e; tot+=a.size(); if(e>0.3*a.size()) bad++;
      if(getenv("SHOW") && e>0.3*a.size()) printf("   [%.0fdB] got \"%s\"\n",snr,b.c_str());
    }
    printf("  %4.0f dB: %5.1f%%   bad runs %d/%d\n",snr,100.0*errs/tot,bad,N);
  }
}
