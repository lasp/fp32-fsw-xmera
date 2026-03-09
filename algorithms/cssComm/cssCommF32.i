%module cssCommF32
%{
   #include "cssComm.h"
   typedef std::array<double, kMaxNumChebyPolys> DoubleArray32;
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>

%template(DoubleArray32) std::array<double, 32>;

%include <attribute.i>
%attribute(CssComm, uint32_t, numSensors, getNumSensors, setNumSensors)
%attribute(CssComm, double, maxSensorValue, getMaxSensorValue, setMaxSensorValue)
%attribute(CssComm, uint32_t, chebyCount, getChebyCount, setChebyCount)

%include "cssComm.h"

%extend CssComm {
%pythoncode %{
    @property
    def kellyCheby(self):
        return self.getChebyPolynomials()

    @kellyCheby.setter
    def kellyCheby(self, value):
        self.setChebyPolynomials(value)
%}
}

%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
