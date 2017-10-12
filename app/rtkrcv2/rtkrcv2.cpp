#include "rtklib.h"

static void PrintUsage( char *argv[] );
extern "C" void updatesvr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int sat,
                          sbsmsg_t *sbsmsg, int index, int iobs);

#define MAXSTR      1024                /* max length of a string */
#define MAXRCVCMD   4096                /* max receiver command */

static int svrcycle     =10;            /* server cycle (ms) */
static int nmeacycle    =5000;          /* nmea request cycle (ms) */
static int buffsize     =32768;         /* input buffer size (bytes) */
static int navmsgsel    =0;             /* navigation mesaage select */
static int nmeareq      =0;             /* nmea request type (0:off,1:lat/lon,2:single) */

static prcopt_t prcopt;                 /* processing options */
static solopt_t solopt[2]={{0}};        /* solution options */
static int strtype[]={                  /* stream types */
    STR_SERIAL,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE
};

static char strpath[8][MAXSTR]={""};    /* stream paths */
static int strfmt[]={                   /* stream formats */
    STRFMT_UBX,STRFMT_RTCM3,STRFMT_SP3,SOLF_LLH,SOLF_NMEA
};

const prcopt_t prcopt_default_my = { /* defaults processing options */
  PMODE_SINGLE,0,2,SYS_GPS,   /* mode,soltype,nf,navsys */
  15.0*D2R,{{0,0}},           /* elmin,snrmask */
  0,1,1,1,                    /* sateph,modear,glomodear,bdsmodear */
  5,0,10,1,                   /* maxout,minlock,minfix,armaxiter */
  0,0,0,0,                    /* estion,esttrop,dynamics,tidecorr */
  1,0,0,0,0,                  /* niter,codesmooth,intpref,sbascorr,sbassatsel */
  0,0,                        /* rovpos,refpos */
  {100.0,100.0},              /* eratio[] */
  {100.0,0.003,0.003,0.0,1.0}, /* err[] */
  {30.0,0.03,0.3},            /* std[] */
  {1E-4,1E-3,1E-4,1E-1,1E-2,0.0}, /* prn[] */
  5E-12,                      /* sclkstab */
  {3.0,0.9999,0.25,0.1,0.05}, /* thresar */
  0.0,0.0,0.05,               /* elmaskar,almaskhold,thresslip */
  30.0,30.0,30.0,             /* maxtdif,maxinno,maxgdop */
  {0},{0},{0},                /* baseline,ru,rb */
  {"",""},                    /* anttype */
  {{0}},{{0}},{0}             /* antdel,pcv,exsats */
};

long long int GetNewEpoch(FILE* f, rtksvr_t *svr, int index){
  long long int time = -1;
  char ch;
  int sat,fobs=0;
  while(fread(&ch,1,1,f))
  {
    obs_t *obs;
    nav_t *nav;
    raw_t * raw = &svr->raw[index];
    sbsmsg_t *sbsmsg=NULL;

    int ret = input_ubx(raw, ch);
    obs=&raw->obs;
    nav=&raw->nav;
    sat=raw->ephsat;
    sbsmsg=&raw->sbsmsg;
    updatesvr(svr,ret,obs,nav,sat,sbsmsg,index,fobs);

    if(ret == 1){
      time = (long long int )raw->time.time*1000 +
              (long long int )(raw->time.sec*1000);
      break;
    }else
      svr->prcout++;
  }
  return time;
}

bool UpdateFirstNav(FILE* f, rtksvr_t *svr, int index)
{
  char ch;
  int sat,fobs=0;
  bool satidx[64];
  memset(satidx, 0,sizeof(satidx));

  while(fread(&ch,1,1,f))
  {
    obs_t *obs;
    nav_t *nav;
    raw_t * raw = &svr->raw[index];
    sbsmsg_t *sbsmsg=NULL;

    int ret = input_ubx(raw, ch);
    if(ret != 2) continue;
    if(satidx[raw->ephsat]) continue;
    satidx[raw->ephsat] = true;
    obs=&raw->obs;
    nav=&raw->nav;
    sat=raw->ephsat;
    sbsmsg=&raw->sbsmsg;
    updatesvr(svr,ret,obs,nav,sat,sbsmsg,index,fobs);
  }
  return true;
}

/* initialize state and covariance -------------------------------------------*/
static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) {
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;
    }
}

#define MAX_PATH 255

int main(int argc, char* argv[])
{
  char file[MAX_PATH]= "";
  char rove_name[MAX_PATH]= "";
  char base_name[MAX_PATH]= "";
  char nmea_name[MAX_PATH]= "";
  if( argc < 2 ){
    PrintUsage( argv );
    return 1;
  }
  strcpy(rove_name, argv[1]);
  for(int i = 2 ; i < argc; i++)
  {
    if((strcmp(argv[i],"-s")==0) && (i < (argc-1))) {
      strcpy(file, argv[++i]);
    }
    else if((strcmp(argv[i],"-b")==0) && (i < (argc-1))){
      strcpy(base_name, argv[++i]);
    }
    else{
      PrintUsage( argv );
      return 2;
    }
  }
  strcpy(nmea_name,rove_name);
  strcat(nmea_name,".nme");

  //extern void traceopen(const char *file)
  char dbg_name[MAX_PATH];
  strcpy(dbg_name,rove_name);
  traceopen(strcat(dbg_name,".dbg"));
  tracelevel(2);
  /*
  char rove_name[] = "D:/_______ASHLEY/logs/27July_Ivanov/27jul_tst2/SPK.log";//"D:/_______ASHLEY/logs/FromIvanov/NAVIS/Test_Ivanov_Dacha/Test_1/21vek.log";
  char base_name[] = "D:/_______ASHLEY/logs/27July_Ivanov/27jul_tst2/REF.log";//"D:/_______ASHLEY/logs/FromIvanov/NAVIS/Test_Ivanov_Dacha/Test_1/reference.log";
  char nmea_name[] = "D:/_______ASHLEY/logs/27July_Ivanov/27jul_tst2/rtklib.nme";//"D:/_______ASHLEY/logs/FromIvanov/NAVIS/Test_Ivanov_Dacha/Test_1/ashley.nme";
  char file[]      = "D:/_______ASHLEY/logs/FromIvanov/NAVIS/Test_Ivanov_Dacha/Test_1/ivanov.conf";
  */
  FILE* rovf = fopen(rove_name, "rb");
  FILE* basf = fopen(base_name, "rb");
  FILE* nmef = fopen(nmea_name, "wb");
  if(!rovf || !basf){
    fprintf(stderr,"Can't open file(s)");
    return 1;
  }

  obs_t obs;
  obsd_t data[MAXOBS*2];
  obs.data=data;

  rtksvr_t* svr = new rtksvr_t;
  memset(svr, 0, sizeof(rtksvr_t));

  if(!rtksvrinit(svr)){
    fprintf(stderr, "Init error\n");
  }

  char *paths[]={
        strpath[0],strpath[1],strpath[2],strpath[3],strpath[4],strpath[5],
        strpath[6],strpath[7]};

  double npos[3];
  char s[3][MAXRCVCMD]={"","",""},*cmds[]={NULL,NULL,NULL};
  char *ropts[]={"","",""};
  char s2[3][MAXRCVCMD] = { "", "", "" }, *cmds_periodic[] = { NULL, NULL, NULL };

  // long long int sizeOfFile;
  // fseek(rovf, 0,SEEK_END);
  // sizeOfFile = ftell(rovf); // to calculate percent of execution
  // fseek(rovf, 0,SEEK_SET);

  rtksvrstart(svr,svrcycle,buffsize,strtype,paths,strfmt,navmsgsel,
    cmds, cmds_periodic,ropts, nmeacycle, nmeareq, npos, &prcopt, solopt, NULL, false);

  prcopt_t prcopt1;                 /* processing options */
  solopt_t solopt1[2]={{0}};        /* solution options */
  filopt_t filopt1  ={""};          /* file options */

  resetsysopts();
  if (!loadopts(file,sysopts)) {
    fprintf(stderr,"no options file: %s. defaults used\n",file);
  }
  getsysopts(&prcopt1, solopt1, &filopt1);

  svr->rtk.opt = prcopt_default_my;//prcopt;//prcopt_default;//prcopt;
  rtkinit(&svr->rtk, &svr->rtk.opt);

  svr->rtk.opt.rb[0] =  2827906.4363335385;//Ivanov country side position
  svr->rtk.opt.rb[1] =  2063390.5069834129;
  svr->rtk.opt.rb[2] =  5313892.2977660177;

  long long int num_sol[3] = {0,0,0};

 // bool ret = UpdateFirstNav(rovf,  svr, 0);
 // _fseeki64(rovf, 10*1024*1024,SEEK_SET);


  long long int rove_time = GetNewEpoch(rovf, svr, 0);
  long long int base_time = GetNewEpoch(basf, svr, 1);

  while(true)
  {
    if(rove_time == -1){
      break; //end of rover file
    }

    //fprintf(stderr, "\r%6.2f%%", 100.0*(_ftelli64(rovf)/(double)sizeOfFile));
    long long int delta = rove_time - base_time;
    if(base_time < 0){
      delta = -2000;
    }
    if((delta < 1000) && (delta > -1000)){
/*
      if(count++%1000 == 0){

        rtk_t *rtk = &svr->rtk;
        rtkfree(&svr->rtk);
        rtkinit(&svr->rtk, &svr->rtk.opt);
        svr->rtk.opt.rb[0] =  2827906.4363335385;
        svr->rtk.opt.rb[1] =  2063390.5069834129;
        svr->rtk.opt.rb[2] =  5313892.2977660177;

        svr->rtk.opt.rb[3] = 0.0;
        svr->rtk.opt.rb[4] = 0.0;
        svr->rtk.opt.rb[5] = 0.0;


        int i;
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
        for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
  //      trace(2,"reset rtk position due to large variance: var=%.3f\n",var);

      }
*/
      //Update rtk
      int j;
      obs.n=0;
      for (j=0;j<svr->obs[0][0].n&&obs.n<MAXOBS*2;j++) {
          obs.data[obs.n++]=svr->obs[0][0].data[j];
      }
      for (j=0;j<svr->obs[1][0].n&&obs.n<MAXOBS*2;j++) {
          obs.data[obs.n++]=svr->obs[1][0].data[j];
      }

      /* rtk positioning */
      rtksvrlock(svr);
      rtkpos(&svr->rtk,obs.data,obs.n,&svr->nav);
      rtksvrunlock(svr);

      if (svr->rtk.sol.stat==SOLQ_FIX) {
        printf("Time: %lld (FIX)\n", rove_time);
        num_sol[0]++;
      }
      else if(svr->rtk.sol.stat==SOLQ_FLOAT) {
        printf("Time: %lld (FLOAT)\n", rove_time);
        num_sol[1]++;
      }
      else
        printf("Time: %lld (ELSE)\n", rove_time);

      num_sol[2]++;

      unsigned char buffer[2048];
      int dwWrite = 0;
      dwWrite += outnmea_rmc(buffer+dwWrite, &svr->rtk.sol);
      dwWrite += outnmea_gga(buffer+dwWrite, &svr->rtk.sol);
      dwWrite += outnmea_gsv(buffer+dwWrite, &svr->rtk.sol, svr->rtk.ssat);
      if(nmef)
        fwrite(buffer, 1, dwWrite, nmef);
      rove_time = GetNewEpoch(rovf, svr, 0);
      base_time = GetNewEpoch(basf, svr, 1);
    }
    else if(delta >= 1000){
      base_time = GetNewEpoch(basf, svr, 1);
    }
    else if(delta <= -1000){
     //Put rover data
      rove_time = GetNewEpoch(rovf, svr, 0);
    }
  }
  //updatesvr
  //int res = rtkpos(&rtk, obs, 0, &nav);
  rtksvrfree(svr);
  printf("Fixed/Float/All %lld/%lld/%lld\n", num_sol[0],num_sol[1],num_sol[2]);
  return 0;
}

/********************************************************************/
static void PrintUsage( char *argv[] )
{
  printf("\nUsage: Ashley <rover> [-b base] [-s options file]\n");
  printf("\n<rover> - is navis rover observation file.\n");
  printf("\n[base]  - is navis base observation file.\n");
  printf("\n[options file] - rtklib options file\n");
  printf("\n\nUtility will create nmea file <rover>.nmea\n");
}
