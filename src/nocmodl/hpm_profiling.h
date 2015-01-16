// copyright EPFL, timothee.ewart@epfl.ch 

// inclusion guard
#ifndef _PROF_HPM_H
#define _PROF_HPM_H
  
// no namespace in C, Oo'
    
    //forward declaration for the counter
    void HPM_helper_include(){
         Fprintf(fcout, "#ifdef _PROF_HPM \n");
         Fprintf(fcout,"void HPM_Start(const char *); \n"); // It is horrible
         Fprintf(fcout,"void HPM_Stop(const char *); \n"); // It is horrible
         Fprintf(fcout, "#endif \n");
    }

    void HPM_helper_start(const char* hpm_name, const char* prefix){
         char hpm_name_with_extension[1024];
         hpm_name_with_extension[0] = '\0';
         strcat(hpm_name_with_extension, hpm_name);
         strcat(hpm_name_with_extension, prefix);
         Fprintf(fcout, "#ifdef _PROF_HPM \n");
         Fprintf(fcout,"HPM_Start(\""); // It is horrible
         Fprintf(fcout, "%s", hpm_name_with_extension);
         Fprintf(fcout,"\"); \n");
         Fprintf(fcout, "#endif \n");
    } 
   
    void HPM_helper_stop(const char* hpm_name, const char* prefix){
         char hpm_name_with_extension[1024];
         hpm_name_with_extension[0] = '\0';
         strcat(hpm_name_with_extension, hpm_name);
         strcat(hpm_name_with_extension, prefix);
         Fprintf(fcout, "#ifdef _PROF_HPM \n");
         Fprintf(fcout,"HPM_Stop(\""); // It is horrible
         Fprintf(fcout, "%s", hpm_name_with_extension);
         Fprintf(fcout,"\"); \n");
         Fprintf(fcout, "#endif \n");
    } 

#endif 
