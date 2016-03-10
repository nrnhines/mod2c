/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
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
 
#define nrn_init _nrn_init__NMDA16_2
#define _nrn_initial _nrn_initial__NMDA16_2
#define nrn_cur _nrn_cur__NMDA16_2
#define _nrn_current _nrn_current__NMDA16_2
#define nrn_jacob _nrn_jacob__NMDA16_2
#define nrn_state _nrn_state__NMDA16_2
#define _net_receive _net_receive__NMDA16_2 
#define kstates kstates__NMDA16_2 
#define rates rates__NMDA16_2 
 
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
#define Erev _p[0]
#define tau _p[1]
#define T_max _p[2]
#define kd1F _p[3]
#define kd1B _p[4]
#define kd2F _p[5]
#define kd2B _p[6]
#define csi _p[7]
#define i _p[8]
#define g _p[9]
#define T _p[10]
#define tRel _p[11]
#define synon _p[12]
#define R _p[13]
#define RA _p[14]
#define RA2 _p[15]
#define RA2d1 _p[16]
#define RA2d2 _p[17]
#define RA2f _p[18]
#define RA2s _p[19]
#define O _p[20]
#define OMg _p[21]
#define RMg _p[22]
#define RAMg _p[23]
#define RA2Mg _p[24]
#define RA2d1Mg _p[25]
#define RA2d2Mg _p[26]
#define RA2fMg _p[27]
#define RA2sMg _p[28]
#define w _p[29]
#define nao _p[30]
#define DR _p[31]
#define DRA _p[32]
#define DRA2 _p[33]
#define DRA2d1 _p[34]
#define DRA2d2 _p[35]
#define DRA2f _p[36]
#define DRA2s _p[37]
#define DO _p[38]
#define DOMg _p[39]
#define DRMg _p[40]
#define DRAMg _p[41]
#define DRA2Mg _p[42]
#define DRA2d1Mg _p[43]
#define DRA2d2Mg _p[44]
#define DRA2fMg _p[45]
#define DRA2sMg _p[46]
#define v _p[47]
#define _g _p[48]
#define _tsav _p[49]
#define _nd_area  *_ppvar[0]._pval
#define _ion_nao	*_ppvar[2]._pval
 
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
 /* declaration of user functions */
 static double _hoc_rates();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "rates", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[6];
#define _gth 2
#define Kcs0 Kcs0_NMDA16_2
 double Kcs0 = 0.27;
#define Kna Kna_NMDA16_2
 double Kna = 34.4;
#define Kcs Kcs_NMDA16_2
 double Kcs = 0;
#define Mg Mg_NMDA16_2
 double Mg = 1;
#define V0 V0_NMDA16_2
 double V0 = -100;
#define Vdep Vdep_NMDA16_2
 double Vdep = 175;
#define a a_NMDA16_2
 double a = -21;
#define b b_NMDA16_2
 double b = -55;
#define c c_NMDA16_2
 double c = 52.7;
#define d d_NMDA16_2
 double d = -50;
#define gmax gmax_NMDA16_2
 double gmax = 50;
#define kNi0 kNi0_NMDA16_2
 double kNi0 = 0.0618;
#define kNo0 kNo0_NMDA16_2
 double kNo0 = 110;
#define kP0 kP0_NMDA16_2
 double kP0 = 1100;
#define kfB0 kfB0_NMDA16_2
 double kfB0 = 0.175;
#define kfF0 kfF0_NMDA16_2
 double kfF0 = 2.836;
#define ksB0 ksB0_NMDA16_2
 double ksB0 = 0.23;
#define ksF0 ksF0_NMDA16_2
 double ksF0 = 0.048;
#define koff koff_NMDA16_2
 double koff = 0.0381;
#define kon kon_NMDA16_2
 double kon = 2.83;
#define kfB_NMDA16_2 _thread1data[0]
#define kfB _thread[_gth]._pval[0]
#define kfF_NMDA16_2 _thread1data[1]
#define kfF _thread[_gth]._pval[1]
#define ksB_NMDA16_2 _thread1data[2]
#define ksB _thread[_gth]._pval[2]
#define ksF_NMDA16_2 _thread1data[3]
#define ksF _thread[_gth]._pval[3]
#define kMgB_NMDA16_2 _thread1data[4]
#define kMgB _thread[_gth]._pval[4]
#define kMgF_NMDA16_2 _thread1data[5]
#define kMgF _thread[_gth]._pval[5]
#define kNi kNi_NMDA16_2
 double kNi = 0;
#define kNo kNo_NMDA16_2
 double kNo = 0;
#define kP kP_NMDA16_2
 double kP = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gmax_NMDA16_2", "pS",
 "Mg_NMDA16_2", "mM",
 "kon_NMDA16_2", "/ms",
 "koff_NMDA16_2", "/ms",
 "ksF0_NMDA16_2", "/ms",
 "ksB0_NMDA16_2", "/ms",
 "kfF0_NMDA16_2", "/ms",
 "kfB0_NMDA16_2", "/ms",
 "Vdep_NMDA16_2", "mV",
 "V0_NMDA16_2", "mV",
 "Kna_NMDA16_2", "mM",
 "Kcs0_NMDA16_2", "mM",
 "a_NMDA16_2", "mV",
 "kP0_NMDA16_2", "/ms",
 "b_NMDA16_2", "mV",
 "kNo0_NMDA16_2", "/ms",
 "c_NMDA16_2", "mV",
 "kNi0_NMDA16_2", "/ms",
 "d_NMDA16_2", "mV",
 "ksF_NMDA16_2", "/ms",
 "ksB_NMDA16_2", "/ms",
 "kfF_NMDA16_2", "/ms",
 "kfB_NMDA16_2", "/ms",
 "kMgF_NMDA16_2", "/ms /mM",
 "kMgB_NMDA16_2", "/ms",
 "Kcs_NMDA16_2", "mM",
 "kP_NMDA16_2", "/ms /mM",
 "kNo_NMDA16_2", "/ms",
 "kNi_NMDA16_2", "/ms",
 "Erev", "mV",
 "tau", "ms",
 "T_max", "mM",
 "kd1F", "/ms",
 "kd1B", "/ms",
 "kd2F", "/ms",
 "kd2B", "/ms",
 "csi", "mM",
 "i", "nA",
 "g", "uS",
 "T", "mM",
 "tRel", "ms",
 0,0
};
 static double OMg0 = 0;
 static double O0 = 0;
 static double RA2sMg0 = 0;
 static double RA2fMg0 = 0;
 static double RA2d2Mg0 = 0;
 static double RA2d1Mg0 = 0;
 static double RA2Mg0 = 0;
 static double RAMg0 = 0;
 static double RMg0 = 0;
 static double RA2s0 = 0;
 static double RA2f0 = 0;
 static double RA2d20 = 0;
 static double RA2d10 = 0;
 static double RA20 = 0;
 static double RA0 = 0;
 static double R0 = 0;
 static double delta_t = 1;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "gmax_NMDA16_2", &gmax_NMDA16_2,
 "Mg_NMDA16_2", &Mg_NMDA16_2,
 "kon_NMDA16_2", &kon_NMDA16_2,
 "koff_NMDA16_2", &koff_NMDA16_2,
 "ksF0_NMDA16_2", &ksF0_NMDA16_2,
 "ksB0_NMDA16_2", &ksB0_NMDA16_2,
 "kfF0_NMDA16_2", &kfF0_NMDA16_2,
 "kfB0_NMDA16_2", &kfB0_NMDA16_2,
 "Vdep_NMDA16_2", &Vdep_NMDA16_2,
 "V0_NMDA16_2", &V0_NMDA16_2,
 "Kna_NMDA16_2", &Kna_NMDA16_2,
 "Kcs0_NMDA16_2", &Kcs0_NMDA16_2,
 "a_NMDA16_2", &a_NMDA16_2,
 "kP0_NMDA16_2", &kP0_NMDA16_2,
 "b_NMDA16_2", &b_NMDA16_2,
 "kNo0_NMDA16_2", &kNo0_NMDA16_2,
 "c_NMDA16_2", &c_NMDA16_2,
 "kNi0_NMDA16_2", &kNi0_NMDA16_2,
 "d_NMDA16_2", &d_NMDA16_2,
 "ksF_NMDA16_2", &ksF_NMDA16_2,
 "ksB_NMDA16_2", &ksB_NMDA16_2,
 "kfF_NMDA16_2", &kfF_NMDA16_2,
 "kfB_NMDA16_2", &kfB_NMDA16_2,
 "kMgF_NMDA16_2", &kMgF_NMDA16_2,
 "kMgB_NMDA16_2", &kMgB_NMDA16_2,
 "Kcs_NMDA16_2", &Kcs_NMDA16_2,
 "kP_NMDA16_2", &kP_NMDA16_2,
 "kNo_NMDA16_2", &kNo_NMDA16_2,
 "kNi_NMDA16_2", &kNi_NMDA16_2,
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
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[4]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"NMDA16_2",
 "Erev",
 "tau",
 "T_max",
 "kd1F",
 "kd1B",
 "kd2F",
 "kd2B",
 "csi",
 0,
 "i",
 "g",
 "T",
 "tRel",
 "synon",
 0,
 "R",
 "RA",
 "RA2",
 "RA2d1",
 "RA2d2",
 "RA2f",
 "RA2s",
 "O",
 "OMg",
 "RMg",
 "RAMg",
 "RA2Mg",
 "RA2d1Mg",
 "RA2d2Mg",
 "RA2fMg",
 "RA2sMg",
 0,
 0};
 static Symbol* _na_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 50, _prop);
 	/*initialize range parameters*/
 	Erev = -0.7;
 	tau = 0.3;
 	T_max = 1.5;
 	kd1F = 0.55;
 	kd1B = 0.081;
 	kd2F = 0.32319;
 	kd2B = 0.00020977;
 	csi = 148;
  }
 	_prop->param = _p;
 	_prop->param_size = 50;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[2]; /* nao */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[3]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _SynNMDA16_2_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 4,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
  _extcall_thread = (Datum*)ecalloc(3, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 50, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "netsend");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 NMDA16_2 /home/ovcharen/dev/mod2c/test/validation/mod/SynNMDA16_2.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Voltage-dependent kinetic model of NMDA receptor";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsprotocomma_ double, double);
 extern double *_nrn_thread_getelm();
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(_so, _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  1
 static int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[16], _dlist1[16]; static double *_temp1;
 static int kstates();
 
static int kstates (void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<16;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v , t ) ;
   /* ~ R <-> RA ( ( 2.0 * kon * T ) , koff )*/
 f_flux =  ( 2.0 * kon * T ) * R ;
 b_flux =  koff * RA ;
 _RHS1( 15) -= (f_flux - b_flux);
 _RHS1( 14) += (f_flux - b_flux);
 
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 14 ,15)  -= _term;
 _term =  koff ;
 _MATELM1( 15 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ RA <-> RA2 ( ( kon * T ) , ( 2.0 * koff ) )*/
 f_flux =  ( kon * T ) * RA ;
 b_flux =  ( 2.0 * koff ) * RA2 ;
 _RHS1( 14) -= (f_flux - b_flux);
 _RHS1( 13) += (f_flux - b_flux);
 
 _term =  ( kon * T ) ;
 _MATELM1( 14 ,14)  += _term;
 _MATELM1( 13 ,14)  -= _term;
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 14 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d1 ( kd1F , kd1B )*/
 f_flux =  kd1F * RA2 ;
 b_flux =  kd1B * RA2d1 ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 12) += (f_flux - b_flux);
 
 _term =  kd1F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  kd1B ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d2 ( kd2F , kd2B )*/
 f_flux =  kd2F * RA2 ;
 b_flux =  kd2B * RA2d2 ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 11) += (f_flux - b_flux);
 
 _term =  kd2F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 11 ,13)  -= _term;
 _term =  kd2B ;
 _MATELM1( 13 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2f ( kfF , kfB )*/
 f_flux =  kfF * RA2 ;
 b_flux =  kfB * RA2f ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 10) += (f_flux - b_flux);
 
 _term =  kfF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 10 ,13)  -= _term;
 _term =  kfB ;
 _MATELM1( 13 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2s ( ksF , ksB )*/
 f_flux =  ksF * RA2 ;
 b_flux =  ksB * RA2s ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 9) += (f_flux - b_flux);
 
 _term =  ksF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 9 ,13)  -= _term;
 _term =  ksB ;
 _MATELM1( 13 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ RA2f <-> O ( ksF , ksB )*/
 f_flux =  ksF * RA2f ;
 b_flux =  ksB * O ;
 _RHS1( 10) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  ksF ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 2 ,10)  -= _term;
 _term =  ksB ;
 _MATELM1( 10 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ RA2s <-> O ( kfF , kfB )*/
 f_flux =  kfF * RA2s ;
 b_flux =  kfB * O ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  kfF ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 2 ,9)  -= _term;
 _term =  kfB ;
 _MATELM1( 9 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ O <-> OMg ( ( kMgF * Mg ) , kMgB )*/
 f_flux =  ( kMgF * Mg ) * O ;
 b_flux =  kMgB * OMg ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  ( kMgF * Mg ) ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  kMgB ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2fMg ( ksB , ksF )*/
 f_flux =  ksB * OMg ;
 b_flux =  ksF * RA2fMg ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  ksB ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 3 ,1)  -= _term;
 _term =  ksF ;
 _MATELM1( 1 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2sMg ( kfB , kfF )*/
 f_flux =  kfB * OMg ;
 b_flux =  kfF * RA2sMg ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  kfB ;
 _MATELM1( 1 ,1)  += _term;
 _term =  kfF ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
  /* ~ RA2fMg <-> RA2Mg ( kfB , kfF )*/
 f_flux =  kfB * RA2fMg ;
 b_flux =  kfF * RA2Mg ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  kfB ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 6 ,3)  -= _term;
 _term =  kfF ;
 _MATELM1( 3 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2sMg <-> RA2Mg ( ksB , ksF )*/
 f_flux =  ksB * RA2sMg ;
 b_flux =  ksF * RA2Mg ;
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  ksB ;
 _MATELM1( 6 ,0)  -= _term;
 _term =  ksF ;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d1Mg ( kd1B , kd1F )*/
 f_flux =  kd1B * RA2Mg ;
 b_flux =  kd1F * RA2d1Mg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  kd1B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  kd1F ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d2Mg ( kd2B , kd2F )*/
 f_flux =  kd2B * RA2Mg ;
 b_flux =  kd2F * RA2d2Mg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  kd2B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 4 ,6)  -= _term;
 _term =  kd2F ;
 _MATELM1( 6 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RAMg ( ( 2.0 * koff ) , ( kon * T ) )*/
 f_flux =  ( 2.0 * koff ) * RA2Mg ;
 b_flux =  ( kon * T ) * RAMg ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 7 ,6)  -= _term;
 _term =  ( kon * T ) ;
 _MATELM1( 6 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ RAMg <-> RMg ( koff , ( 2.0 * kon * T ) )*/
 f_flux =  koff * RAMg ;
 b_flux =  ( 2.0 * kon * T ) * RMg ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  koff ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 8 ,7)  -= _term;
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 7 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
   /* R + RA + RA2 + RA2d1 + RA2d2 + RA2f + RA2s + O + OMg + RMg + RAMg + RA2Mg + RA2d1Mg + RA2d2Mg + RA2fMg + RA2sMg = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= RA2sMg ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= RA2fMg ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= RA2d2Mg ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= RA2d1Mg ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= RA2Mg ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= RAMg ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= RMg ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= OMg ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= O ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= RA2s ;
 _MATELM1(0, 10) = 1;
 _RHS1(0) -= RA2f ;
 _MATELM1(0, 11) = 1;
 _RHS1(0) -= RA2d2 ;
 _MATELM1(0, 12) = 1;
 _RHS1(0) -= RA2d1 ;
 _MATELM1(0, 13) = 1;
 _RHS1(0) -= RA2 ;
 _MATELM1(0, 14) = 1;
 _RHS1(0) -= RA ;
 _MATELM1(0, 15) = 1;
 _RHS1(0) -= R ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   if ( _lflag  == 0.0 ) {
     tRel = t ;
     synon = 1.0 ;
     w = _args[0] ;
     }
   if ( _lflag  == 1.0 ) {
     R = 1.0 ;
     RA = 0.0 ;
     RA2 = 0.0 ;
     RA2d1 = 0.0 ;
     RA2d2 = 0.0 ;
     RA2f = 0.0 ;
     RA2s = 0.0 ;
     O = 0.0 ;
     OMg = 0.0 ;
     RMg = 0.0 ;
     RAMg = 0.0 ;
     RA2Mg = 0.0 ;
     RA2d1Mg = 0.0 ;
     RA2d2Mg = 0.0 ;
     RA2fMg = 0.0 ;
     RA2sMg = 0.0 ;
     }
   } }
 
static int  rates ( _threadargsprotocomma_ double _lv , double _lt ) {
   T = T_max * ( _lt - tRel ) / tau * exp ( 1.0 - ( _lt - tRel ) / tau ) * synon ;
   kMgF = 610e-3 * exp ( 1.0 * - _lv / 17.0 ) * 1.0 ;
   kMgB = 5400e-3 * exp ( 1.0 * _lv / 47.0 ) * 1.0 ;
   ksF = ksF0 * exp ( ( _lv - V0 ) / Vdep ) ;
   ksB = ksB0 * exp ( ( _lv - V0 ) / Vdep * ( - 1.0 ) ) ;
   kfF = kfF0 * exp ( ( _lv - V0 ) / Vdep ) ;
   kfB = kfB0 * exp ( ( _lv - V0 ) / Vdep * ( - 1.0 ) ) ;
    return 0; }
 
static double _hoc_rates(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 return(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<16;_i++) _p[_dlist1[_i]] = 0.0;}
 rates ( _threadargscomma_ v , t ) ;
 /* ~ R <-> RA ( ( 2.0 * kon * T ) , koff )*/
 f_flux =  ( 2.0 * kon * T ) * R ;
 b_flux =  koff * RA ;
 DR -= (f_flux - b_flux);
 DRA += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA <-> RA2 ( ( kon * T ) , ( 2.0 * koff ) )*/
 f_flux =  ( kon * T ) * RA ;
 b_flux =  ( 2.0 * koff ) * RA2 ;
 DRA -= (f_flux - b_flux);
 DRA2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2d1 ( kd1F , kd1B )*/
 f_flux =  kd1F * RA2 ;
 b_flux =  kd1B * RA2d1 ;
 DRA2 -= (f_flux - b_flux);
 DRA2d1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2d2 ( kd2F , kd2B )*/
 f_flux =  kd2F * RA2 ;
 b_flux =  kd2B * RA2d2 ;
 DRA2 -= (f_flux - b_flux);
 DRA2d2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2f ( kfF , kfB )*/
 f_flux =  kfF * RA2 ;
 b_flux =  kfB * RA2f ;
 DRA2 -= (f_flux - b_flux);
 DRA2f += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2 <-> RA2s ( ksF , ksB )*/
 f_flux =  ksF * RA2 ;
 b_flux =  ksB * RA2s ;
 DRA2 -= (f_flux - b_flux);
 DRA2s += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2f <-> O ( ksF , ksB )*/
 f_flux =  ksF * RA2f ;
 b_flux =  ksB * O ;
 DRA2f -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2s <-> O ( kfF , kfB )*/
 f_flux =  kfF * RA2s ;
 b_flux =  kfB * O ;
 DRA2s -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ O <-> OMg ( ( kMgF * Mg ) , kMgB )*/
 f_flux =  ( kMgF * Mg ) * O ;
 b_flux =  kMgB * OMg ;
 DO -= (f_flux - b_flux);
 DOMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ OMg <-> RA2fMg ( ksB , ksF )*/
 f_flux =  ksB * OMg ;
 b_flux =  ksF * RA2fMg ;
 DOMg -= (f_flux - b_flux);
 DRA2fMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ OMg <-> RA2sMg ( kfB , kfF )*/
 f_flux =  kfB * OMg ;
 b_flux =  kfF * RA2sMg ;
 DOMg -= (f_flux - b_flux);
 DRA2sMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2fMg <-> RA2Mg ( kfB , kfF )*/
 f_flux =  kfB * RA2fMg ;
 b_flux =  kfF * RA2Mg ;
 DRA2fMg -= (f_flux - b_flux);
 DRA2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2sMg <-> RA2Mg ( ksB , ksF )*/
 f_flux =  ksB * RA2sMg ;
 b_flux =  ksF * RA2Mg ;
 DRA2sMg -= (f_flux - b_flux);
 DRA2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d1Mg ( kd1B , kd1F )*/
 f_flux =  kd1B * RA2Mg ;
 b_flux =  kd1F * RA2d1Mg ;
 DRA2Mg -= (f_flux - b_flux);
 DRA2d1Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d2Mg ( kd2B , kd2F )*/
 f_flux =  kd2B * RA2Mg ;
 b_flux =  kd2F * RA2d2Mg ;
 DRA2Mg -= (f_flux - b_flux);
 DRA2d2Mg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RA2Mg <-> RAMg ( ( 2.0 * koff ) , ( kon * T ) )*/
 f_flux =  ( 2.0 * koff ) * RA2Mg ;
 b_flux =  ( kon * T ) * RAMg ;
 DRA2Mg -= (f_flux - b_flux);
 DRAMg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RAMg <-> RMg ( koff , ( 2.0 * kon * T ) )*/
 f_flux =  koff * RAMg ;
 b_flux =  ( 2.0 * kon * T ) * RMg ;
 DRAMg -= (f_flux - b_flux);
 DRMg += (f_flux - b_flux);
 
 /*REACTION*/
   /* R + RA + RA2 + RA2d1 + RA2d2 + RA2f + RA2s + O + OMg + RMg + RAMg + RA2Mg + RA2d1Mg + RA2d2Mg + RA2fMg + RA2sMg = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<16;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v , t ) ;
 /* ~ R <-> RA ( ( 2.0 * kon * T ) , koff )*/
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 14 ,15)  -= _term;
 _term =  koff ;
 _MATELM1( 15 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ RA <-> RA2 ( ( kon * T ) , ( 2.0 * koff ) )*/
 _term =  ( kon * T ) ;
 _MATELM1( 14 ,14)  += _term;
 _MATELM1( 13 ,14)  -= _term;
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 14 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d1 ( kd1F , kd1B )*/
 _term =  kd1F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  kd1B ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2d2 ( kd2F , kd2B )*/
 _term =  kd2F ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 11 ,13)  -= _term;
 _term =  kd2B ;
 _MATELM1( 13 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2f ( kfF , kfB )*/
 _term =  kfF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 10 ,13)  -= _term;
 _term =  kfB ;
 _MATELM1( 13 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ RA2 <-> RA2s ( ksF , ksB )*/
 _term =  ksF ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 9 ,13)  -= _term;
 _term =  ksB ;
 _MATELM1( 13 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ RA2f <-> O ( ksF , ksB )*/
 _term =  ksF ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 2 ,10)  -= _term;
 _term =  ksB ;
 _MATELM1( 10 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ RA2s <-> O ( kfF , kfB )*/
 _term =  kfF ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 2 ,9)  -= _term;
 _term =  kfB ;
 _MATELM1( 9 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ O <-> OMg ( ( kMgF * Mg ) , kMgB )*/
 _term =  ( kMgF * Mg ) ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  kMgB ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2fMg ( ksB , ksF )*/
 _term =  ksB ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 3 ,1)  -= _term;
 _term =  ksF ;
 _MATELM1( 1 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ OMg <-> RA2sMg ( kfB , kfF )*/
 _term =  kfB ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  kfF ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ RA2fMg <-> RA2Mg ( kfB , kfF )*/
 _term =  kfB ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 6 ,3)  -= _term;
 _term =  kfF ;
 _MATELM1( 3 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2sMg <-> RA2Mg ( ksB , ksF )*/
 _term =  ksB ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 6 ,0)  -= _term;
 _term =  ksF ;
 _MATELM1( 0 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d1Mg ( kd1B , kd1F )*/
 _term =  kd1B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  kd1F ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RA2d2Mg ( kd2B , kd2F )*/
 _term =  kd2B ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 4 ,6)  -= _term;
 _term =  kd2F ;
 _MATELM1( 6 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ RA2Mg <-> RAMg ( ( 2.0 * koff ) , ( kon * T ) )*/
 _term =  ( 2.0 * koff ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 7 ,6)  -= _term;
 _term =  ( kon * T ) ;
 _MATELM1( 6 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ RAMg <-> RMg ( koff , ( 2.0 * kon * T ) )*/
 _term =  koff ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 8 ,7)  -= _term;
 _term =  ( 2.0 * kon * T ) ;
 _MATELM1( 7 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
   /* R + RA + RA2 + RA2d1 + RA2d2 + RA2f + RA2s + O + OMg + RMg + RAMg + RA2Mg + RA2d1Mg + RA2d2Mg + RA2fMg + RA2sMg = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 16;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  nao = _ion_nao;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 16; ++_i) {
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
  nao = _ion_nao;
 _cvode_sparse_thread(&_thread[_cvspth1]._pvoid, 16, _dlist1, _p, _ode_matsol1, _ppvar, _thread, _nt);
 }}
 
static void _thread_mem_init(Datum* _thread) {
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(6, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(_thread[_cvspth1]._pvoid);
   _nrn_destroy_sparseobj_thread(_thread[_spth1]._pvoid);
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 2);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  OMg = OMg0;
  O = O0;
  RA2sMg = RA2sMg0;
  RA2fMg = RA2fMg0;
  RA2d2Mg = RA2d2Mg0;
  RA2d1Mg = RA2d1Mg0;
  RA2Mg = RA2Mg0;
  RAMg = RAMg0;
  RMg = RMg0;
  RA2s = RA2s0;
  RA2f = RA2f0;
  RA2d2 = RA2d20;
  RA2d1 = RA2d10;
  RA2 = RA20;
  RA = RA0;
  R = R0;
 {
   T = 0.0 ;
   synon = 0.0 ;
   tRel = 0.0 ;
   R = 1.0 ;
   rates ( _threadargscomma_ v , t ) ;
   net_send ( _tqitem, (double*)0, _ppvar[1]._pvoid, t +  590.0 , 1.0 ) ;
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
 _tsav = -1e20;
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
  nao = _ion_nao;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   g = w * gmax * O ;
   i = g * ( v - Erev ) ;
   }
 _current += i;

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
  nao = _ion_nao;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
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
  nao = _ion_nao;
 {  sparse_thread(&_thread[_spth1]._pvoid, 16, _slist1, _dlist1, _p, &t, dt, kstates, _linmat1, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(RA2sMg) - _p;  _dlist1[0] = &(DRA2sMg) - _p;
 _slist1[1] = &(OMg) - _p;  _dlist1[1] = &(DOMg) - _p;
 _slist1[2] = &(O) - _p;  _dlist1[2] = &(DO) - _p;
 _slist1[3] = &(RA2fMg) - _p;  _dlist1[3] = &(DRA2fMg) - _p;
 _slist1[4] = &(RA2d2Mg) - _p;  _dlist1[4] = &(DRA2d2Mg) - _p;
 _slist1[5] = &(RA2d1Mg) - _p;  _dlist1[5] = &(DRA2d1Mg) - _p;
 _slist1[6] = &(RA2Mg) - _p;  _dlist1[6] = &(DRA2Mg) - _p;
 _slist1[7] = &(RAMg) - _p;  _dlist1[7] = &(DRAMg) - _p;
 _slist1[8] = &(RMg) - _p;  _dlist1[8] = &(DRMg) - _p;
 _slist1[9] = &(RA2s) - _p;  _dlist1[9] = &(DRA2s) - _p;
 _slist1[10] = &(RA2f) - _p;  _dlist1[10] = &(DRA2f) - _p;
 _slist1[11] = &(RA2d2) - _p;  _dlist1[11] = &(DRA2d2) - _p;
 _slist1[12] = &(RA2d1) - _p;  _dlist1[12] = &(DRA2d1) - _p;
 _slist1[13] = &(RA2) - _p;  _dlist1[13] = &(DRA2) - _p;
 _slist1[14] = &(RA) - _p;  _dlist1[14] = &(DRA) - _p;
 _slist1[15] = &(R) - _p;  _dlist1[15] = &(DR) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
