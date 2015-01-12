/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#ifdef _PROF_HPM 
void HPM_Start(const char *); 
void HPM_Stop(const char *); 
#endif 
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__cc
#define _nrn_initial _nrn_initial__cc
#define nrn_cur _nrn_cur__cc
#define _nrn_current _nrn_current__cc
#define nrn_jacob _nrn_jacob__cc
#define nrn_state _nrn_state__cc
#define _net_receive _net_receive__cc 
#define settables settables__cc 
#define states states__cc 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gcapbar _p[0]
#define captempfactor _p[1]
#define gcatbar _p[2]
#define cattempfactor _p[3]
#define gkvcbar _p[4]
#define gkcbar _p[5]
#define gcap _p[6]
#define gcat _p[7]
#define gkvc _p[8]
#define gkc _p[9]
#define mpcal _p[10]
#define hpcal _p[11]
#define mtcal _p[12]
#define htcal _p[13]
#define okvc _p[14]
#define okc _p[15]
#define cai _p[16]
#define eca _p[17]
#define ica _p[18]
#define ek _p[19]
#define ik _p[20]
#define mpcalinf _p[21]
#define hpcalinf _p[22]
#define mpcaltau _p[23]
#define hpcaltau _p[24]
#define mtcalinf _p[25]
#define htcalinf _p[26]
#define mtcaltau _p[27]
#define htcaltau _p[28]
#define Dmpcal _p[29]
#define Dhpcal _p[30]
#define Dmtcal _p[31]
#define Dhtcal _p[32]
#define Dokvc _p[33]
#define Dokc _p[34]
#define v _p[35]
#define _g _p[36]
#define _ion_eca	*_ppvar[0]._pval
#define _ion_cai	*_ppvar[1]._pval
#define _ion_ica	*_ppvar[2]._pval
#define _ion_dicadv	*_ppvar[3]._pval
#define _ion_ek	*_ppvar[4]._pval
#define _ion_ik	*_ppvar[5]._pval
#define _ion_dikdv	*_ppvar[6]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_okctau(void);
 static void _hoc_okvctau(void);
 static void _hoc_okcinf(void);
 static void _hoc_okvcinf(void);
 static void _hoc_settables(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_cc", _hoc_setdata,
 "okctau_cc", _hoc_okctau,
 "okvctau_cc", _hoc_okvctau,
 "okcinf_cc", _hoc_okcinf,
 "okvcinf_cc", _hoc_okvcinf,
 "settables_cc", _hoc_settables,
 0, 0
};
#define okctau okctau_cc
#define okvctau okvctau_cc
#define okcinf okcinf_cc
#define okvcinf okvcinf_cc
 extern double okctau( _threadargsprotocomma_ double );
 extern double okvctau( _threadargsprotocomma_ double );
 extern double okcinf( _threadargsprotocomma_ double );
 extern double okvcinf( _threadargsprotocomma_ double );
 /* declare global and static user variables */
#define b b_cc
 double b = 2.5;
#define cal_base cal_base_cc
 double cal_base = 3e-06;
#define htcalinfslopechange htcalinfslopechange_cc
 double htcalinfslopechange = 0;
#define htcalinfhalfshift htcalinfhalfshift_cc
 double htcalinfhalfshift = 0;
#define hpcalinfslopechange hpcalinfslopechange_cc
 double hpcalinfslopechange = 0;
#define hpcalinfhalfshift hpcalinfhalfshift_cc
 double hpcalinfhalfshift = 0;
#define mtcalinfslopechange mtcalinfslopechange_cc
 double mtcalinfslopechange = 0;
#define mtcalinfhalfshift mtcalinfhalfshift_cc
 double mtcalinfhalfshift = 0;
#define mpcalinfslopechange mpcalinfslopechange_cc
 double mpcalinfslopechange = 0;
#define mpcalinfhalfshift mpcalinfhalfshift_cc
 double mpcalinfhalfshift = 0;
#define okvctempfactor okvctempfactor_cc
 double okvctempfactor = 1;
#define okvcslopechange okvcslopechange_cc
 double okvcslopechange = 0;
#define okvcvhalfshift okvcvhalfshift_cc
 double okvcvhalfshift = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gcapbar_cc", "S/cm2",
 "gcatbar_cc", "S/cm2",
 "gkvcbar_cc", "S/cm2",
 "gkcbar_cc", "S/cm2",
 "gcap_cc", "S/cm2",
 "gcat_cc", "S/cm2",
 "gkvc_cc", "S/cm2",
 "gkc_cc", "S/cm2",
 0,0
};
 static double delta_t = 0.01;
 static double htcal0 = 0;
 static double hpcal0 = 0;
 static double mtcal0 = 0;
 static double mpcal0 = 0;
 static double okc0 = 0;
 static double okvc0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "mpcalinfhalfshift_cc", &mpcalinfhalfshift_cc,
 "mpcalinfslopechange_cc", &mpcalinfslopechange_cc,
 "hpcalinfhalfshift_cc", &hpcalinfhalfshift_cc,
 "hpcalinfslopechange_cc", &hpcalinfslopechange_cc,
 "mtcalinfhalfshift_cc", &mtcalinfhalfshift_cc,
 "mtcalinfslopechange_cc", &mtcalinfslopechange_cc,
 "htcalinfhalfshift_cc", &htcalinfhalfshift_cc,
 "htcalinfslopechange_cc", &htcalinfslopechange_cc,
 "okvcvhalfshift_cc", &okvcvhalfshift_cc,
 "okvcslopechange_cc", &okvcslopechange_cc,
 "okvctempfactor_cc", &okvctempfactor_cc,
 "cal_base_cc", &cal_base_cc,
 "b_cc", &b_cc,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[7]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"cc",
 "gcapbar_cc",
 "captempfactor_cc",
 "gcatbar_cc",
 "cattempfactor_cc",
 "gkvcbar_cc",
 "gkcbar_cc",
 0,
 "gcap_cc",
 "gcat_cc",
 "gkvc_cc",
 "gkc_cc",
 0,
 "mpcal_cc",
 "hpcal_cc",
 "mtcal_cc",
 "htcal_cc",
 "okvc_cc",
 "okc_cc",
 0,
 0};
 static Symbol* _ca_sym;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 37, _prop);
 	/*initialize range parameters*/
 	gcapbar = 0;
 	captempfactor = 1;
 	gcatbar = 0;
 	cattempfactor = 1;
 	gkvcbar = 0;
 	gkcbar = 0;
 	_prop->param = _p;
 	_prop->param_size = 37;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 8, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 1);
 	_ppvar[0]._pval = &prop_ion->param[0]; /* eca */
 	_ppvar[1]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[4]._pval = &prop_ion->param[0]; /* ek */
 	_ppvar[5]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[6]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _new_calcium_channels_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", -10000.);
 	ion_reg("k", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 37, 8);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 cc /home/yates/project/Buildyard/src/Neurodamus/build/lib/modlib/x86_64/new_calcium_channels.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96.4853;
 static double R = 8.31342;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int settables(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[6], _dlist1[6];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   double _linf , _ltau ;
 settables ( _threadargscomma_ v ) ;
   Dmpcal = ( ( mpcalinf - mpcal ) / mpcaltau ) ;
   Dhpcal = ( ( hpcalinf - hpcal ) / hpcaltau ) ;
   Dmtcal = ( ( mtcalinf - mtcal ) / mtcaltau ) ;
   Dhtcal = ( ( htcalinf - htcal ) / htcaltau ) ;
   _linf = okvcinf ( _threadargscomma_ v ) ;
   _ltau = okvctau ( _threadargscomma_ v ) ;
   Dokvc = ( _linf - okvc ) / _ltau ;
   _linf = okcinf ( _threadargscomma_ v ) ;
   _ltau = okctau ( _threadargscomma_ v ) ;
   Dokc = ( _linf - okc ) / _ltau ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 double _linf , _ltau ;
 settables ( _threadargscomma_ v ) ;
 Dmpcal = Dmpcal  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / mpcaltau ) )) ;
 Dhpcal = Dhpcal  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / hpcaltau ) )) ;
 Dmtcal = Dmtcal  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / mtcaltau ) )) ;
 Dhtcal = Dhtcal  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / htcaltau ) )) ;
 _linf = okvcinf ( _threadargscomma_ v ) ;
 _ltau = okvctau ( _threadargscomma_ v ) ;
 Dokvc = Dokvc  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _ltau )) ;
 _linf = okcinf ( _threadargscomma_ v ) ;
 _ltau = okctau ( _threadargscomma_ v ) ;
 Dokc = Dokc  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _ltau )) ;
 return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   double _linf , _ltau ;
 settables ( _threadargscomma_ v ) ;
    mpcal = mpcal + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / mpcaltau ))))*(- ( ( ( ( mpcalinf ) ) / mpcaltau ) ) / ( ( ( ( ( - 1.0) ) ) / mpcaltau ) ) - mpcal) ;
    hpcal = hpcal + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / hpcaltau ))))*(- ( ( ( ( hpcalinf ) ) / hpcaltau ) ) / ( ( ( ( ( - 1.0) ) ) / hpcaltau ) ) - hpcal) ;
    mtcal = mtcal + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / mtcaltau ))))*(- ( ( ( ( mtcalinf ) ) / mtcaltau ) ) / ( ( ( ( ( - 1.0) ) ) / mtcaltau ) ) - mtcal) ;
    htcal = htcal + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / htcaltau ))))*(- ( ( ( ( htcalinf ) ) / htcaltau ) ) / ( ( ( ( ( - 1.0) ) ) / htcaltau ) ) - htcal) ;
   _linf = okvcinf ( _threadargscomma_ v ) ;
   _ltau = okvctau ( _threadargscomma_ v ) ;
    okvc = okvc + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _ltau)))*(- ( ( ( _linf ) ) / _ltau ) / ( ( ( ( - 1.0) ) ) / _ltau ) - okvc) ;
   _linf = okcinf ( _threadargscomma_ v ) ;
   _ltau = okctau ( _threadargscomma_ v ) ;
    okc = okc + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _ltau)))*(- ( ( ( _linf ) ) / _ltau ) / ( ( ( ( - 1.0) ) ) / _ltau ) - okc) ;
   }
  return 0;
}
 
static int  settables ( _threadargsprotocomma_ double _lv ) {
   mpcalinf = 1.0 / ( 1.0 + exp ( - ( ( _lv + 57.0 + mpcalinfhalfshift ) / ( 6.2 + mpcalinfslopechange ) ) ) ) ;
   hpcalinf = 1.0 / ( 1.0 + exp ( - ( ( _lv + 81.0 + hpcalinfhalfshift ) / ( 4.0 + hpcalinfslopechange ) ) ) ) ;
   mpcaltau = ( 0.612 + ( 1.0 / ( exp ( - ( _lv + 132.0 ) / 16.7 ) + exp ( ( _lv + 16.8 ) / 18.2 ) ) ) ) * captempfactor ;
   if ( _lv < - 80.0 ) {
     hpcaltau = ( exp ( ( _lv + 467.0 ) / 66.6 ) ) * captempfactor ;
     }
   else {
     hpcaltau = ( 28.0 + exp ( - ( _lv + 22.0 ) / 10.5 ) ) * captempfactor ;
     }
   mtcalinf = 1.0 / ( 1.0 + exp ( - ( ( _lv + 57.0 + mtcalinfhalfshift ) / ( 6.2 + mtcalinfslopechange ) ) ) ) ;
   htcalinf = 1.0 / ( 1.0 + exp ( ( _lv + 81.0 + htcalinfhalfshift ) / ( 4.0 + htcalinfslopechange ) ) ) ;
   mtcaltau = ( 0.612 + ( 1.0 / ( exp ( - ( _lv + 132.0 ) / 16.7 ) + exp ( ( _lv + 16.8 ) / 18.2 ) ) ) ) * cattempfactor ;
   if ( _lv < - 80.0 ) {
     htcaltau = ( exp ( ( _lv + 467.0 ) / 66.6 ) ) * cattempfactor ;
     }
   else {
     htcaltau = ( 28.0 + exp ( - ( _lv + 22.0 ) / 10.5 ) ) * cattempfactor ;
     }
    return 0; }
 
static void _hoc_settables(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 settables ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double okvcinf ( _threadargsprotocomma_ double _lVm ) {
   double _lokvcinf;
  _lokvcinf = ( cai / ( cai + cal_base ) ) * 1.0 / ( 1.0 + exp ( - ( ( _lVm + 28.3 + okvcvhalfshift ) / ( 12.6 + okvcslopechange ) ) ) ) ;
    
return _lokvcinf;
 }
 
static void _hoc_okvcinf(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  okvcinf ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double okvctau ( _threadargsprotocomma_ double _lVm ) {
   double _lokvctau;
  _lokvctau = ( 90.3 - ( 75.1 / ( 1.0 + exp ( - ( _lVm + 46.0 ) / 22.7 ) ) ) ) * okvctempfactor ;
    
return _lokvctau;
 }
 
static void _hoc_okvctau(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  okvctau ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double okcinf ( _threadargsprotocomma_ double _lVm ) {
   double _lokcinf;
 double _la ;
  _la = 1.25 * ( pow( 10.0 , 8.0 ) ) * ( cai ) * ( cai ) ;
   _lokcinf = _la / ( _la + b ) ;
    
return _lokcinf;
 }
 
static void _hoc_okcinf(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  okcinf ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double okctau ( _threadargsprotocomma_ double _lVm ) {
   double _lokctau;
 double _la ;
  _la = 1.25 * ( pow( 10.0 , 8.0 ) ) * ( cai ) * ( cai ) ;
   _lokctau = 1000.0 / ( _la + b ) ;
    
return _lokctau;
 }
 
static void _hoc_okctau(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  okctau ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 6;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eca = _ion_eca;
  cai = _ion_cai;
  ek = _ion_ek;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
   }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 6; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eca = _ion_eca;
  cai = _ion_cai;
  ek = _ion_ek;
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 0);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 3, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 4, 0);
   nrn_update_ion_pointer(_k_sym, _ppvar, 5, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 6, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  htcal = htcal0;
  hpcal = hpcal0;
  mtcal = mtcal0;
  mpcal = mpcal0;
  okc = okc0;
  okvc = okvc0;
 {
   settables ( _threadargscomma_ v ) ;
   mpcal = mpcalinf ;
   hpcal = hpcalinf ;
   mtcal = mtcalinf ;
   htcal = htcalinf ;
   okvc = okvcinf ( _threadargscomma_ v ) ;
   okc = okcinf ( _threadargscomma_ v ) ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  eca = _ion_eca;
  cai = _ion_cai;
  ek = _ion_ek;
 initmodel(_p, _ppvar, _thread, _nt);
  }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   gcap = gcapbar * mpcal * mpcal * hpcal ;
   gcat = gcatbar * mtcal * mtcal * htcal ;
   gkvc = gkvcbar * ( pow( okvc , 4.0 ) ) ;
   gkc = gkcbar * okc * okc ;
   ica = ( gcap + gcat ) * ( v - eca ) ;
   ik = ( gkvc + gkc ) * ( v - ek ) ;
   }
 _current += ica;
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  eca = _ion_eca;
  cai = _ion_cai;
  ek = _ion_ek;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
 double _dica;
  _dica = ica;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dicadv += (_dica - ica)/.001 ;
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
#ifdef _PROF_HPM 
HPM_Start("nrn_state_new_calcium_channels"); 
#endif 
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  eca = _ion_eca;
  cai = _ion_cai;
  ek = _ion_ek;
 {   states(_p, _ppvar, _thread, _nt);
  }  }}
#ifdef _PROF_HPM 
HPM_Stop("nrn_state_new_calcium_channels"); 
#endif 

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(mpcal) - _p;  _dlist1[0] = &(Dmpcal) - _p;
 _slist1[1] = &(hpcal) - _p;  _dlist1[1] = &(Dhpcal) - _p;
 _slist1[2] = &(mtcal) - _p;  _dlist1[2] = &(Dmtcal) - _p;
 _slist1[3] = &(htcal) - _p;  _dlist1[3] = &(Dhtcal) - _p;
 _slist1[4] = &(okvc) - _p;  _dlist1[4] = &(Dokvc) - _p;
 _slist1[5] = &(okc) - _p;  _dlist1[5] = &(Dokc) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
