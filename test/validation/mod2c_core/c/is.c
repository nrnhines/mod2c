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
 
#define nrn_init _nrn_init__Is
#define nrn_cur _nrn_cur__Is
#define _nrn_current _nrn_current__Is
#define nrn_jacob _nrn_jacob__Is
#define nrn_state _nrn_state__Is
#define _net_receive _net_receive__Is 
#define rates rates__Is 
#define states states__Is 
 
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
#define gsbar _p[0*_STRIDE]
#define ica _p[1*_STRIDE]
#define ics _p[2*_STRIDE]
#define m _p[3*_STRIDE]
#define n _p[4*_STRIDE]
#define Dm _p[5*_STRIDE]
#define Dn _p[6*_STRIDE]
#define cai _p[7*_STRIDE]
#define ins _p[8*_STRIDE]
#define lca _p[9*_STRIDE]
#define _v_unused _p[10*_STRIDE]
#define _g_unused _p[11*_STRIDE]
#define _ion_cai		_nt_data[_ppvar[0*_STRIDE]]
#define _ion_ica	_nt_data[_ppvar[1*_STRIDE]]
#define _ion_dicadv	_nt_data[_ppvar[2*_STRIDE]]
#define _ion_ics	_nt_data[_ppvar[3*_STRIDE]]
#define _ion_dicsdv	_nt_data[_ppvar[4*_STRIDE]]
#define _ion_ins	_nt_data[_ppvar[5*_STRIDE]]
#define _ion_dinsdv	_nt_data[_ppvar[6*_STRIDE]]
 
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
 extern double celsius;
 
#if 0 /*BBCORE*/
 /* declaration of user functions */
 static void _hoc_alp(void);
 static void _hoc_bet(void);
 static void _hoc_rates(void);
 
#endif /*BBCORE*/
 static int _mechtype;
extern int nrn_get_mechtype();
extern void hoc_register_prop_size(int, int, int);
extern Memb_func* memb_func;
 
#if 0 /*BBCORE*/
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_Is", _hoc_setdata,
 "alp_Is", _hoc_alp,
 "bet_Is", _hoc_bet,
 "rates_Is", _hoc_rates,
 0, 0
};
 
#endif /*BBCORE*/
#define alp alp_Is
#define bet bet_Is
 inline double alp( _threadargsprotocomma_ double , double );
 inline double bet( _threadargsprotocomma_ double , double );
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[4];
#define _gth 4
#define mtau_Is _thread1data[0]
#define mtau _thread[_gth]._pval[0]
#define minf_Is _thread1data[1]
#define minf _thread[_gth]._pval[1]
#define ntau_Is _thread1data[2]
#define ntau _thread[_gth]._pval[2]
#define ninf_Is _thread1data[3]
#define ninf _thread[_gth]._pval[3]
 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "gsbar_Is", 0, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "mtau_Is", "ms",
 "ntau_Is", "ms",
 "gsbar_Is", "S/cm2",
 "ica_Is", "mA/cm2",
 "ics_Is", "mA/cm2",
 0,0
};
 
#endif /*BBCORE*/
 static double delta_t = 0.01;
 static double m0 = 0;
 static double n0 = 0;
 
#if 0 /*BBCORE*/
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "minf_Is", &minf_Is,
 "ninf_Is", &ninf_Is,
 "mtau_Is", &mtau_Is,
 "ntau_Is", &ntau_Is,
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
"Is",
 "gsbar_Is",
 0,
 "ica_Is",
 "ics_Is",
 0,
 "m_Is",
 "n_Is",
 0,
 0};
 static int _ca_type;
 static int _cs_type;
 static int _ns_type;
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	gsbar = 5e-05;
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 prop_ion = need_memb(_cs_sym);
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ics */
 	_ppvar[4]._pval = &prop_ion->param[4]; /* _ion_dicsdv */
 prop_ion = need_memb(_ns_sym);
 	_ppvar[5]._pval = &prop_ion->param[3]; /* ins */
 	_ppvar[6]._pval = &prop_ion->param[4]; /* _ion_dinsdv */
 
#endif /* BBCORE */
 
}
 static void _initlists();
 static void _thread_mem_init(ThreadDatum*);
 static void _thread_cleanup(ThreadDatum*);
 static void _update_ion_pointer(Datum*);
 
#define _psize 12
#define _ppsize 7
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(_threadargsproto_, int));
extern void _cvode_abstol( Symbol**, double*, int);

 void _is_reg() {
	int _vectorized = 1;
  _initlists();
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 _ca_type = nrn_get_mechtype("ca_ion"); _cs_type = nrn_get_mechtype("cs_ion"); _ns_type = nrn_get_mechtype("ns_ion"); 
#if 0 /*BBCORE*/
 	ion_reg("ca", -10000.);
 	ion_reg("cs", 2.0);
 	ion_reg("ns", 2.0);
 	_ca_sym = hoc_lookup("ca_ion");
 	_cs_sym = hoc_lookup("cs_ion");
 	_ns_sym = hoc_lookup("ns_ion");
 
#endif /*BBCORE*/
 	register_mech(_mechanism, nrn_alloc,nrn_cur, NULL, nrn_state, nrn_init, hoc_nrnpointerindex, 6);
  _extcall_thread = (ThreadDatum*)ecalloc(5, sizeof(ThreadDatum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
     _nrn_thread_reg1(_mechtype, _thread_mem_init);
     _nrn_thread_reg0(_mechtype, _thread_cleanup);
  hoc_register_prop_size(_mechtype, _psize, _ppsize);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cs_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cs_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "ns_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "ns_ion");
 }
static char *modelname = "Cardiac L-type Calcium channel";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsprotocomma_ double);
 
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
   rates ( _threadargscomma_ v ) ;
   Dm = ( minf - m ) / mtau ;
   Dn = ( ninf - n ) / ntau ;
   }
 return _reset;
}
 static int _ode_matsol1 (_threadargsproto_) {
 rates ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / mtau )) ;
 Dn = Dn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / ntau )) ;
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
   rates ( _threadargscomma_ v ) ;
   Dm = ( minf - m ) / mtau ;
   Dn = ( ninf - n ) / ntau ;
   {int _id; for(_id=0; _id < 2; _id++) {
if (_deriv1_advance) {
 _dlist2[++_counte] = _p[_dlist1[_id]] - (_p[_slist1[_id]] - _savstate1[_id])/dt;
 }else{
_dlist2[++_counte] = _p[_slist1[_id]] - _savstate1[_id];}}}
 } }
 return _reset;}
 
double alp ( _threadargsprotocomma_ double _lv , double _li ) {
   double _lalp;
 if ( _li  == 0.0 ) {
     _lalp = 0.095 * exp ( - 0.01 * ( _lv - 5.0 ) ) / ( exp ( - 0.072 * ( _lv - 5.0 ) ) + 1.0 ) ;
     }
   else if ( _li  == 1.0 ) {
     _lalp = 0.012 * exp ( - 0.008 * ( _lv + 28.0 ) ) / ( exp ( 0.15 * ( _lv + 28.0 ) ) + 1.0 ) ;
     }
   
return _lalp;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_alp(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  alp ( _threadargs_, *getarg(1) , *getarg(2) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
double bet ( _threadargsprotocomma_ double _lv , double _li ) {
   double _lbet;
 if ( _li  == 0.0 ) {
     _lbet = 0.07 * exp ( - 0.017 * ( _lv + 44.0 ) ) / ( exp ( 0.05 * ( _lv + 44.0 ) ) + 1.0 ) ;
     }
   else if ( _li  == 1.0 ) {
     _lbet = 0.0065 * exp ( - 0.02 * ( _lv + 30.0 ) ) / ( exp ( - 0.2 * ( _lv + 30.0 ) ) + 1.0 ) ;
     }
   
return _lbet;
 }
 
#if 0 /*BBCORE*/
 
static void _hoc_bet(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  bet ( _threadargs_, *getarg(1) , *getarg(2) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
static int  rates ( _threadargsprotocomma_ double _lv ) {
   double _la , _lb ;
 _la = alp ( _threadargscomma_ _lv , 0.0 ) ;
   _lb = bet ( _threadargscomma_ _lv , 0.0 ) ;
   mtau = 1.0 / ( _la + _lb ) ;
   minf = _la / ( _la + _lb ) ;
   _la = alp ( _threadargscomma_ _lv , 1.0 ) ;
   _lb = bet ( _threadargscomma_ _lv , 1.0 ) ;
   ntau = 1.0 / ( _la + _lb ) ;
   ninf = _la / ( _la + _lb ) ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static void _hoc_rates(void) {
  double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 rates ( _threadargs_, *getarg(1) ;
 hoc_retpushx(_r);
}
 
#endif /*BBCORE*/
 
static void _thread_mem_init(ThreadDatum* _thread) {
   _thread[_dith1]._pval = (double*)ecalloc(4, sizeof(double));
   _newtonspace1 = nrn_cons_newtonspace(2);
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(4, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(ThreadDatum* _thread) {
   free((void*)(_thread[_dith1]._pval));
   nrn_destroy_newtonspace(_newtonspace1);
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }
 static void _update_ion_pointer(Datum* _ppvar) {
 }

static void initmodel(_threadargsproto_) {
  int _i; double _save;{
  m = m0;
  n = n0;
 {
   rates ( _threadargscomma_ v ) ;
   m = minf ;
   n = ninf ;
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
  cai = _ion_cai;
 initmodel(_threadargs_);
   }
}

static double _nrn_current(_threadargsproto_, double _v){double _current=0.;v=_v;{ {
   double _lEs ;
 _lEs = - 82.3 - 13.0287 * log ( cai ) ;
   ics = gsbar * m * n * ( v - _lEs ) ;
   ica = ics ;
   ins = - ics ;
   }
 _current += ica;
 _current += ics;
 _current += ins;

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
  cai = _ion_cai;
 _g = _nrn_current(_threadargs_, _v + .001);
 	{ double _dins;
 double _dics;
 double _dica;
  _dica = ica;
  _dics = ics;
  _dins = ins;
 _rhs = _nrn_current(_threadargs_, _v);
  _ion_dicadv += (_dica - ica)/.001 ;
  _ion_dicsdv += (_dics - ics)/.001 ;
  _ion_dinsdv += (_dins - ins)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
  _ion_ics += ics ;
  _ion_ins += ins ;
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
  cai = _ion_cai;
 {  _deriv1_advance = 1;
 derivimplicit_thread(2, _slist1, _dlist1, _p, states, _ppvar, _thread, _nt);
_deriv1_advance = 0;
  }   }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
 int _cntml=0;
 int _iml=0;
  if (!_first) return;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
 _slist1[1] = &(n) - _p;  _dlist1[1] = &(Dn) - _p;
 _slist2[0] = &(m) - _p;
 _slist2[1] = &(n) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
