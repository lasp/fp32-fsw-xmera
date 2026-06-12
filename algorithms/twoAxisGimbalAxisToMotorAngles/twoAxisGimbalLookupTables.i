%module twoAxisGimbalLookupTables
%{
   #include "twoAxisGimbalLookupTables.h"
%}

%include <swig_conly_data.i>

%include <std_array.i>

%template(Array81Double) std::array<double, 76>;
%template(Array111x81Double) std::array<std::array<double, 76>, 111>;
%template(Array320Double) std::array<double, 320>;
%template(Array320x320Double) std::array<std::array<double, 320>, 320>;

%include "twoAxisGimbalLookupTables.h"
