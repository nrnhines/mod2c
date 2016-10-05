/* Created by Language version: 6.2.0 */
/* NOT VECTORIZED */
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
 
#define nrn_init _nrn_init__NMDA10_2
#define _nrn_initial _nrn_initial__NMDA10_2
#define nrn_cur _nrn_cur__NMDA10_2
#define _nrn_current _nrn_current__NMDA10_2
#define nrn_jacob _nrn_jacob__NMDA10_2
#define nrn_state _nrn_state__NMDA10_2
#define _net_receive _net_receive__NMDA10_2 
#define kstates kstates__NMDA10_2 
#define release release__NMDA10_2 
#define rates rates__NMDA10_2 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define Erev _p[0]
#define gmax _p[1]
#define tau _p[2]
#define T_max _p[3]
#define i _p[4]
#define g _p[5]
#define rb _p[6]
#define rbb _p[7]
#define RMgB _p[8]
#define RMgU _p[9]
#define T _p[10]
#define tRel _p[11]
#define synon _p[12]
#define C0 _p[13]
#define C1 _p[14]
#define C2 _p[15]
#define D _p[16]
#define O _p[17]
#define CB0 _p[18]
#define CB1 _p[19]
#define CB2 _p[20]
#define DB _p[21]
#define OB _p[22]
#define w _p[23]
#define DC0 _p[24]
#define DC1 _p[25]
#define DC2 _p[26]
#define DD _p[27]
#define DO _p[28]
#define DCB0 _p[29]
#define DCB1 _p[30]
#define DCB2 _p[31]
#define DDB _p[32]
#define DOB _p[33]
#define _g _p[34]
#define _tsav _p[35]
#define _nd_area  *_ppvar[0]._pval
 
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
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_release();
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
 _p = _prop->param; _ppvar = _prop->dparam;
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
 "release", _hoc_release,
 "rates", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
#define Rc Rc_NMDA10_2
 double Rc = 0.0916;
#define Ro Ro_NMDA10_2
 double Ro = 0.0465;
#define Rr Rr_NMDA10_2
 double Rr = 0.0018;
#define Rd Rd_NMDA10_2
 double Rd = 0.0084;
#define Ru Ru_NMDA10_2
 double Ru = 0.0055;
#define Rb Rb_NMDA10_2
 double Rb = 5;
#define mg mg_NMDA10_2
 double mg = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "mg_NMDA10_2", "mM",
 "Rb_NMDA10_2", "/mM",
 "Ru_NMDA10_2", "/ms",
 "Rd_NMDA10_2", "/ms",
 "Rr_NMDA10_2", "/ms",
 "Ro_NMDA10_2", "/ms",
 "Rc_NMDA10_2", "/ms",
 "Erev", "mV",
 "gmax", "pS",
 "tau", "ms",
 "T_max", "mM",
 "i", "nA",
 "g", "uS",
 "rb", "/ms",
 "rbb", "/ms",
 "RMgB", "/ms",
 "RMgU", "/ms",
 "T", "mM",
 "tRel", "ms",
 0,0
};
 static double CB20 = 0;
 static double CB10 = 0;
 static double CB00 = 0;
 static double C20 = 0;
 static double C10 = 0;
 static double C00 = 0;
 static double DB0 = 0;
 static double D0 = 0;
 static double OB0 = 0;
 static double O0 = 0;
 static double delta_t = 1;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "mg_NMDA10_2", &mg_NMDA10_2,
 "Rb_NMDA10_2", &Rb_NMDA10_2,
 "Ru_NMDA10_2", &Ru_NMDA10_2,
 "Rd_NMDA10_2", &Rd_NMDA10_2,
 "Rr_NMDA10_2", &Rr_NMDA10_2,
 "Ro_NMDA10_2", &Ro_NMDA10_2,
 "Rc_NMDA10_2", &Rc_NMDA10_2,
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
 
#define _cvode_ieq _ppvar[2]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"NMDA10_2",
 "Erev",
 "gmax",
 "tau",
 "T_max",
 0,
 "i",
 "g",
 "rb",
 "rbb",
 "RMgB",
 "RMgU",
 "T",
 "tRel",
 "synon",
 0,
 "C0",
 "C1",
 "C2",
 "D",
 "O",
 "CB0",
 "CB1",
 "CB2",
 "DB",
 "OB",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 36, _prop);
 	/*initialize range parameters*/
 	Erev = -0.7;
 	gmax = 50;
 	tau = 0.3;
 	T_max = 1.5;
  }
 	_prop->param = _p;
 	_prop->param_size = 36;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _SynNMDA10_2_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 36, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 NMDA10_2 /home/ovcharen/dev/mod2c/test/validation/mod/SynNMDA10_2.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "detailed model of glutamate NMDA receptors";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int release(double);
static int rates(double);
 extern double *_getelm();
 
#define _MATELM1(_row,_col)	*(_getelm(_row + 1, _col + 1))
 
#define _RHS1(_arg) _coef1[_arg + 1]
 static double *_coef1;
 
#define _linmat1  1
 static void* _sparseobj1;
 static void* _cvsparseobj1;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[10], _dlist1[10]; static double *_temp1;
 static int kstates();
 
static int kstates ()
 {_reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<10;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 release ( _threadargscomma_ t ) ;
   rb = Rb * T ;
   rates ( _threadargscomma_ v ) ;
   /* ~ C0 <-> C1 ( ( 2.0 * rb ) , Ru )*/
 f_flux =  ( 2.0 * rb ) * C0 ;
 b_flux =  Ru * C1 ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  Ru ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ C1 <-> C2 ( rb , ( 2.0 * Ru ) )*/
 f_flux =  rb * C1 ;
 b_flux =  ( 2.0 * Ru ) * C2 ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  rb ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 4 ,5)  -= _term;
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 5 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ C2 <-> D ( Rd , Rr )*/
 f_flux =  Rd * C2 ;
 b_flux =  Rr * D ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  Rd ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 8 ,4)  -= _term;
 _term =  Rr ;
 _MATELM1( 4 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ C2 <-> O ( Ro , Rc )*/
 f_flux =  Ro * C2 ;
 b_flux =  Rc * O ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 9) += (f_flux - b_flux);
 
 _term =  Ro ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 9 ,4)  -= _term;
 _term =  Rc ;
 _MATELM1( 4 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ O <-> OB ( RMgB , RMgU )*/
 f_flux =  RMgB * O ;
 b_flux =  RMgU * OB ;
 _RHS1( 9) -= (f_flux - b_flux);
 
 _term =  RMgB ;
 _MATELM1( 9 ,9)  += _term;
 _term =  RMgU ;
 _MATELM1( 9 ,0)  -= _term;
 /*REACTION*/
  /* ~ OB <-> CB2 ( ( 3.0 * Rc ) , Ro )*/
 f_flux =  ( 3.0 * Rc ) * OB ;
 b_flux =  Ro * CB2 ;
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  ( 3.0 * Rc ) ;
 _MATELM1( 1 ,0)  -= _term;
 _term =  Ro ;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> DB ( Rd , Rr )*/
 f_flux =  Rd * CB2 ;
 b_flux =  Rr * DB ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  Rd ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 7 ,1)  -= _term;
 _term =  Rr ;
 _MATELM1( 1 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> CB1 ( ( 2.0 * Ru ) , rb )*/
 f_flux =  ( 2.0 * Ru ) * CB2 ;
 b_flux =  rb * CB1 ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 2 ,1)  -= _term;
 _term =  rb ;
 _MATELM1( 1 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ CB1 <-> CB0 ( Ru , ( 2.0 * rb ) )*/
 f_flux =  Ru * CB1 ;
 b_flux =  ( 2.0 * rb ) * CB0 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  Ru ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
   /* C0 + C1 + C2 + D + O + CB0 + CB1 + CB2 + DB + OB = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= OB ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= DB ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= CB2 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= CB1 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= CB0 ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= O ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= D ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= C2 ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= C1 ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= C0 ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   if ( _lflag  == 0.0 ) {
     tRel = t ;
     synon = 1.0 ;
     w = _args[0] ;
     }
   } }
 
static int  release (  double _lt ) {
   T = T_max * ( _lt - tRel ) / tau * exp ( 1.0 - ( _lt - tRel ) / tau ) * synon ;
   
/*VERBATIM*/
	return 0;
  return 0; }
 
static double _hoc_release(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r = 1.;
 release (  *getarg(1) );
 return(_r);
}
 
static int  rates (  double _lv ) {
   RMgB = 610e-3 * exp ( 1.0 * - _lv / 17.0 ) * ( mg / 1.0 ) * 1.0 ;
   RMgU = 5400e-3 * exp ( 1.0 * _lv / 47.0 ) * 1.0 ;
    return 0; }
 
static double _hoc_rates(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r = 1.;
 rates (  *getarg(1) );
 return(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<10;_i++) _p[_dlist1[_i]] = 0.0;}
 release ( _threadargscomma_ t ) ;
 rb = Rb * T ;
 rates ( _threadargscomma_ v ) ;
 /* ~ C0 <-> C1 ( ( 2.0 * rb ) , Ru )*/
 f_flux =  ( 2.0 * rb ) * C0 ;
 b_flux =  Ru * C1 ;
 DC0 -= (f_flux - b_flux);
 DC1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ C1 <-> C2 ( rb , ( 2.0 * Ru ) )*/
 f_flux =  rb * C1 ;
 b_flux =  ( 2.0 * Ru ) * C2 ;
 DC1 -= (f_flux - b_flux);
 DC2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ C2 <-> D ( Rd , Rr )*/
 f_flux =  Rd * C2 ;
 b_flux =  Rr * D ;
 DC2 -= (f_flux - b_flux);
 DD += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ C2 <-> O ( Ro , Rc )*/
 f_flux =  Ro * C2 ;
 b_flux =  Rc * O ;
 DC2 -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ O <-> OB ( RMgB , RMgU )*/
 f_flux =  RMgB * O ;
 b_flux =  RMgU * OB ;
 DO -= (f_flux - b_flux);
 DOB += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ OB <-> CB2 ( ( 3.0 * Rc ) , Ro )*/
 f_flux =  ( 3.0 * Rc ) * OB ;
 b_flux =  Ro * CB2 ;
 DOB -= (f_flux - b_flux);
 DCB2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ CB2 <-> DB ( Rd , Rr )*/
 f_flux =  Rd * CB2 ;
 b_flux =  Rr * DB ;
 DCB2 -= (f_flux - b_flux);
 DDB += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ CB2 <-> CB1 ( ( 2.0 * Ru ) , rb )*/
 f_flux =  ( 2.0 * Ru ) * CB2 ;
 b_flux =  rb * CB1 ;
 DCB2 -= (f_flux - b_flux);
 DCB1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ CB1 <-> CB0 ( Ru , ( 2.0 * rb ) )*/
 f_flux =  Ru * CB1 ;
 b_flux =  ( 2.0 * rb ) * CB0 ;
 DCB1 -= (f_flux - b_flux);
 DCB0 += (f_flux - b_flux);
 
 /*REACTION*/
   /* C0 + C1 + C2 + D + O + CB0 + CB1 + CB2 + DB + OB = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<10;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 release ( _threadargscomma_ t ) ;
 rb = Rb * T ;
 rates ( _threadargscomma_ v ) ;
 /* ~ C0 <-> C1 ( ( 2.0 * rb ) , Ru )*/
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  Ru ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ C1 <-> C2 ( rb , ( 2.0 * Ru ) )*/
 _term =  rb ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 4 ,5)  -= _term;
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 5 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ C2 <-> D ( Rd , Rr )*/
 _term =  Rd ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 8 ,4)  -= _term;
 _term =  Rr ;
 _MATELM1( 4 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ C2 <-> O ( Ro , Rc )*/
 _term =  Ro ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 9 ,4)  -= _term;
 _term =  Rc ;
 _MATELM1( 4 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ O <-> OB ( RMgB , RMgU )*/
 _term =  RMgB ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 0 ,9)  -= _term;
 _term =  RMgU ;
 _MATELM1( 9 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ OB <-> CB2 ( ( 3.0 * Rc ) , Ro )*/
 _term =  ( 3.0 * Rc ) ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 1 ,0)  -= _term;
 _term =  Ro ;
 _MATELM1( 0 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> DB ( Rd , Rr )*/
 _term =  Rd ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 7 ,1)  -= _term;
 _term =  Rr ;
 _MATELM1( 1 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ CB2 <-> CB1 ( ( 2.0 * Ru ) , rb )*/
 _term =  ( 2.0 * Ru ) ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 2 ,1)  -= _term;
 _term =  rb ;
 _MATELM1( 1 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ CB1 <-> CB0 ( Ru , ( 2.0 * rb ) )*/
 _term =  Ru ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  ( 2.0 * rb ) ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
   /* C0 + C1 + C2 + D + O + CB0 + CB1 + CB2 + DB + OB = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 10;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 10; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _cvode_sparse(&_cvsparseobj1, 10, _dlist1, _p, _ode_matsol1, &_coef1);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  CB2 = CB20;
  CB1 = CB10;
  CB0 = CB00;
  C2 = C20;
  C1 = C10;
  C0 = C00;
  DB = DB0;
  D = D0;
  OB = OB0;
  O = O0;
 {
   T = 0.0 ;
   synon = 0.0 ;
   tRel = 0.0 ;
   rates ( _threadargscomma_ v ) ;
   C0 = 1.0 ;
   C1 = 0.0 ;
   C2 = 0.0 ;
   D = 0.0 ;
   O = 0.0 ;
   CB0 = 0.0 ;
   CB1 = 0.0 ;
   CB2 = 0.0 ;
   DB = 0.0 ;
   OB = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   g = w * gmax * O ;
   i = g * ( v - Erev ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 _g = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
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
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 { error = sparse(&_sparseobj1, 10, _slist1, _dlist1, _p, &t, dt, kstates,&_coef1, _linmat1);
 if(error){fprintf(stderr,"at line 142 in file SynNMDA10_2.mod:\n\n"); nrn_complain(_p); abort_run(error);}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(OB) - _p;  _dlist1[0] = &(DOB) - _p;
 _slist1[1] = &(CB2) - _p;  _dlist1[1] = &(DCB2) - _p;
 _slist1[2] = &(CB1) - _p;  _dlist1[2] = &(DCB1) - _p;
 _slist1[3] = &(CB0) - _p;  _dlist1[3] = &(DCB0) - _p;
 _slist1[4] = &(C2) - _p;  _dlist1[4] = &(DC2) - _p;
 _slist1[5] = &(C1) - _p;  _dlist1[5] = &(DC1) - _p;
 _slist1[6] = &(C0) - _p;  _dlist1[6] = &(DC0) - _p;
 _slist1[7] = &(DB) - _p;  _dlist1[7] = &(DDB) - _p;
 _slist1[8] = &(D) - _p;  _dlist1[8] = &(DD) - _p;
 _slist1[9] = &(O) - _p;  _dlist1[9] = &(DO) - _p;
_first = 0;
}
