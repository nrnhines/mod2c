/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "coreneuron/mech/cfile/scoplib.h"
#undef PI
 
#include "coreneuron/nrnoc/md1redef.h"
#include "coreneuron/nrnconf.h"
#include "coreneuron/nrnoc/multicore.h"

#include "coreneuron/utils/randoms/nrnran123.h"

#include "coreneuron/nrnoc/md2redef.h"
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#if !defined(DISABLE_HOC_EXP)
#undef exp
#define exp hoc_Exp
#endif
extern double hoc_Exp(double);
#endif
 
#if defined(__clang__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("clang loop vectorize(enable)")
#elif defined(__ICC) || defined(__INTEL_COMPILER)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("ivdep")
#elif defined(__IBMC__) || defined(__IBMCPP__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("ibm independent_loop")
#elif defined(__PGI)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("vector")
#elif defined(_CRAYC)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("_CRI ivdep")
#elif defined(__GNUC__) || defined(__GNUG__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("GCC ivdep")
#else
#define _PRAGMA_FOR_VECTOR_LOOP_
#endif // _PRAGMA_FOR_VECTOR_LOOP_
 
#if !defined(LAYOUT)
/* 1 means AoS, >1 means AoSoA, <= 0 means SOA */
#define LAYOUT 1
#endif
#if LAYOUT >= 1
#define _STRIDE LAYOUT
#else
#define _STRIDE _cntml + _iml
#endif
 
#define nrn_init _nrn_init__NapDA
#define nrn_cur _nrn_cur__NapDA
#define _nrn_current _nrn_current__NapDA
#define nrn_jacob _nrn_jacob__NapDA
#define nrn_state _nrn_state__NapDA
#define _net_receive _net_receive__NapDA 
#define states states__NapDA 
 
#define _threadargscomma_ _iml, _cntml, _p, _ppvar, _thread, _nt, v,
#define _threadargsprotocomma_ int _iml, int _cntml, double* _p, Datum* _ppvar, ThreadDatum* _thread, _NrnThread* _nt, double v,
#define _threadargs_ _iml, _cntml, _p, _ppvar, _thread, _nt, v
#define _threadargsproto_ int _iml, int _cntml, double* _p, Datum* _ppvar, ThreadDatum* _thread, _NrnThread* _nt, double v
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gNapbar _p[0*_STRIDE]
#define ena _p[1*_STRIDE]
#define gna _p[2*_STRIDE]
#define m _p[3*_STRIDE]
#define h _p[4*_STRIDE]
#define ina _p[5*_STRIDE]
#define Dm _p[6*_STRIDE]
#define Dh _p[7*_STRIDE]
#define _v_unused _p[8*_STRIDE]
#define _g_unused _p[9*_STRIDE]
#define _ion_ina	_nt_data[_ppvar[0*_STRIDE]]
#define _ion_dinadv	_nt_data[_ppvar[1*_STRIDE]]
 
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
 static ThreadDatum* _extcall_thread;
 /* external NEURON variables */
 
#if 0 /*BBCORE*/
 /* declaration of user functions */
 static void _hoc_hbet(void);
 static void _hoc_half(void);
 static void _hoc_mbet(void);
 static void _hoc_malf(void);
 
#endif /*BBCORE*/
 static int _mechtype;
extern int nrn_get_mechtype();
extern void hoc_register_prop_size(int, int, int);
extern Memb_func* memb_func;
 
#if 0 /*BBCORE*/
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_NapDA", _hoc_setdata,
 "hbet_NapDA", _hoc_hbet,
 "half_NapDA", _hoc_half,
 "mbet_NapDA", _hoc_mbet,
 "malf_NapDA", _hoc_malf,
 0, 0
};
 
#endif /*BBCORE*/
#define hbet hbet_NapDA
#define half half_NapDA
#define mbet mbet_NapDA
#define malf malf_NapDA
 inline double hbet( _threadargsprotocomma_ double );
 inline double half( _threadargsprotocomma_ double );
 inline double mbet( _threadargsprotocomma_ double );
 inline double malf( _threadargsprotocomma_ double );
 /* declare global and static user variables */
 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "gNapbar_NapDA", 0, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gNapbar_NapDA", "mho/cm2",
 "ena_NapDA", "mV",
 "gna_NapDA", "mho/cm2",
 0,0
};
 
#endif /*BBCORE*/
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
 
#if 0 /*BBCORE*/
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 
#endif /*BBCORE*/
 static double _sav_indep;
 static void nrn_alloc(double*, Datum*, int);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"NapDA",
 "gNapbar_NapDA",
 "ena_NapDA",
 0,
 "gna_NapDA",
 0,
 "m_NapDA",
 "h_NapDA",
 0,
 0};
 static int _na_type;
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	gNapbar = 0.0022;
 	ena = 55;
 prop_ion = need_memb(_na_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 
#endif /* BBCORE */
 
}
 static void _initlists();
 static void _thread_mem_init(ThreadDatum*);
 static void _thread_cleanup(ThreadDatum*);
 static void _update_ion_pointer(Datum*);
 
#define _psize 10
#define _ppsize 2
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(_threadargsproto_, int));
extern void _cvode_abstol( Symbol**, double*, int);

 void _NapDA_reg() {
	int _vectorized = 1;
  _initlists();
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 _na_type = nrn_get_mechtype("na_ion"); 
#if 0 /*BBCORE*/
 	ion_reg("na", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 
#endif /*BBCORE*/
 	register_mech(_mechanism, nrn_alloc,nrn_cur, NULL, nrn_state, nrn_init, hoc_nrnpointerindex, 5);
  _extcall_thread = (ThreadDatum*)ecalloc(4, sizeof(ThreadDatum));
  _thread_mem_init(_extcall_thread);
     _nrn_thread_reg1(_mechtype, _thread_mem_init);
     _nrn_thread_reg0(_mechtype, _thread_cleanup);
  hoc_register_prop_size(_mechtype, _psize, _ppsize);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
 }
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
#define _deriv1_advance _thread[0]._i
#define _dith1 1
#define _recurse _thread[2]._i
#define _newtonspace1 _thread[3]._pvoid
extern void* nrn_cons_newtonspace(int);
 
static int _ode_spec1(_threadargsproto_);
static int _ode_matsol1(_threadargsproto_);
 static int _slist2[2];
  static int _slist1[2], _dlist1[2];
 static inline int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_threadargsproto_) {int _reset = 0; {
   Dm = ( 1.0 - m ) * malf ( _threadargscomma_ v ) - m * mbet ( _threadargscomma_ v ) ;
   Dh = ( ( 1.0 - h ) * half ( _threadargscomma_ v ) - h * hbet ( _threadargscomma_ v ) ) / 2.0 ;
   }
 return _reset;
}
 static int _ode_matsol1 (_threadargsproto_) {
 Dm = Dm  / (1. - dt*( (( ( - 1.0 ) ))*(malf ( _threadargscomma_ v )) - (1.0)*(mbet ( _threadargscomma_ v )) )) ;
 Dh = Dh  / (1. - dt*( ( ( (( ( - 1.0 ) ))*(half ( _threadargscomma_ v )) - (1.0)*(hbet ( _threadargscomma_ v )) ) ) / 2.0 )) ;
 return 0;
}
 /*END CVODE*/
 
static int states (_threadargsproto_) {int _reset=0; int error = 0;
 { double* _savstate1 = _thread[_dith1]._pval;
 double* _dlist2 = _thread[_dith1]._pval + 2;
 int _counte = -1;
 if (!_recurse) {
 _recurse = 1;
 {int _id; for(_id=0; _id < 2; _id++) { _savstate1[_id] = _p[_slist1[_id]];}}
 error = nrn_newton_thread(_newtonspace1, 2,_slist2, _p, states, _dlist2, _ppvar, _thread, _nt);
 _recurse = 0; if(error) {abort_run(error);}}
 {
   Dm = ( 1.0 - m ) * malf ( _threadargscomma_ v ) - m * mbet ( _threadargscomma_ v ) ;
   Dh = ( ( 1.0 - h ) * half ( _threadargscomma_ v ) - h * hbet ( _threadargscomma_ v ) ) / 2.0 ;
   {int _id; for(_id=0; _id < 2; _id++) {
if (_deriv1_advance) {
 _dlist2[++_counte] = _p[_dlist1[_id]] - (_p[_slist1[_id]] - _savstate1[_id])/dt;
 }else{
_dlist2[++_counte] = _p[_slist1[_id]] - _savstate1[_id];}}}
 } }
 return _reset;}
 
double malf ( _threadargsprotocomma_ double _lv ) {
   double _lmalf;
 double _lva ;
 _lva = _lv + 12.0 ;
   if ( fabs ( _lva ) < 1e-04 ) {
     _lmalf = - 0.2816 * ( - 9.3 - _lva * 0.5 ) ;
     }
   else {
     _lmalf = - 0.2816 * _lva / ( - 1.0 + exp ( - _lva / 9.3 ) ) ;
     }
   
return _lmalf;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_malf(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  malf ( _threadargs_, *getarg(1) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
double mbet ( _threadargsprotocomma_ double _lv ) {
   double _lmbet;
 double _lvb ;
 _lvb = _lv - 15.0 ;
   if ( fabs ( _lvb ) < 1e-04 ) {
     _lmbet = 0.2464 * ( 6.0 - _lvb * 0.5 ) ;
     }
   else {
     _lmbet = 0.2464 * _lvb / ( - 1.0 + exp ( _lvb / 6.0 ) ) ;
     }
   
return _lmbet;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_mbet(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  mbet ( _threadargs_, *getarg(1) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
double half ( _threadargsprotocomma_ double _lv ) {
   double _lhalf;
 _lhalf = 2.8e-5 * ( exp ( - ( _lv + 42.8477 ) / 4.0248 ) ) ;
   
return _lhalf;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_half(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  half ( _threadargs_, *getarg(1) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
double hbet ( _threadargsprotocomma_ double _lv ) {
   double _lhbet;
 _lhbet = 0.02 / ( 1.0 + exp ( - ( _lv - 413.9284 ) / 148.2589 ) ) ;
   
return _lhbet;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_hbet(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  hbet ( _threadargs_, *getarg(1) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
static void _thread_mem_init(ThreadDatum* _thread) {
   _thread[_dith1]._pval = (double*)ecalloc(4, sizeof(double));
   _newtonspace1 = nrn_cons_newtonspace(2);
 }
 
static void _thread_cleanup(ThreadDatum* _thread) {
   free((void*)(_thread[_dith1]._pval));
   nrn_destroy_newtonspace(_newtonspace1);
 }
 static void _update_ion_pointer(Datum* _ppvar) {
 }

static void initmodel(_threadargsproto_) {
  int _i; double _save;{
  h = h0;
  m = m0;
 {
   m = malf ( _threadargscomma_ v ) / ( malf ( _threadargscomma_ v ) + mbet ( _threadargscomma_ v ) ) ;
   h = half ( _threadargscomma_ v ) / ( half ( _threadargscomma_ v ) + hbet ( _threadargscomma_ v ) ) ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; ThreadDatum* _thread;
double _v, v; int* _ni; int _iml, _cntml;
    _ni = _ml->_nodeindices;
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#endif
#if LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
for (_iml = 0; _iml < _cntml; ++_iml) {
#endif
#if LAYOUT > 1 /*AoSoA*/
#error AoSoA not implemented.
#endif
    int _nd_idx = _ni[_iml];
    _v = _vec_v[_nd_idx];
 v = _v;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_threadargsproto_, double _v){double _current=0.;v=_v;{ {
   gna = gNapbar * m * h ;
   ina = gna * ( v - ena ) ;
   }
 _current += ina;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; ThreadDatum* _thread;
int* _ni; double _rhs, _g, _v, v; int _iml, _cntml;
    _ni = _ml->_nodeindices;
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
double * _vec_rhs = _nt->_actual_rhs;
double * _vec_d = _nt->_actual_d;
double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#endif
#if LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
/* insert compiler dependent ivdep like pragma */
_PRAGMA_FOR_VECTOR_LOOP_
for (_iml = 0; _iml < _cntml; ++_iml) {
#endif
#if LAYOUT > 1 /*AoSoA*/
#error AoSoA not implemented.
#endif
    int _nd_idx = _ni[_iml];
    _v = _vec_v[_nd_idx];
 _g = _nrn_current(_threadargs_, _v + .001);
 	{ double _dina;
  _dina = ina;
 _rhs = _nrn_current(_threadargs_, _v);
  _ion_dinadv += (_dina - ina)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina ;
	_vec_rhs[_nd_idx] -= _rhs;
	_vec_d[_nd_idx] += _g;
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; ThreadDatum* _thread;
double v, _v = 0.0; int* _ni; int _iml, _cntml;
    _ni = _ml->_nodeindices;
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#endif
#if LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
/* insert compiler dependent ivdep like pragma */
_PRAGMA_FOR_VECTOR_LOOP_
for (_iml = 0; _iml < _cntml; ++_iml) {
#endif
#if LAYOUT > 1 /*AoSoA*/
#error AoSoA not implemented.
#endif
    int _nd_idx = _ni[_iml];
    _v = _vec_v[_nd_idx];
 v=_v;
{
 {  _deriv1_advance = 1;
 derivimplicit_thread(2, _slist1, _dlist1, _p, states, _ppvar, _thread, _nt);
_deriv1_advance = 0;
  } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
 int _cntml=0;
 int _iml=0;
  if (!_first) return;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
 _slist1[1] = &(h) - _p;  _dlist1[1] = &(Dh) - _p;
 _slist2[0] = &(h) - _p;
 _slist2[1] = &(m) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
