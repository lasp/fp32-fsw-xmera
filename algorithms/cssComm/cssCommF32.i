%module cssCommF32
%{
   #include "cssComm.h"
   typedef std::array<float, kMaxNumChebyPolys> FloatArray10;
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>

%template(FloatArray10) std::array<float, 10>;

%include <attribute.i>
%attribute(CssComm, uint32_t, numSensors, getNumSensors, setNumSensors)
%attribute(CssComm, float, maxSensorValue, getMaxSensorValue, setMaxSensorValue)
%attribute(CssComm, uint32_t, chebyCount, getChebyCount, setChebyCount)

%include "cssComm.h"
%include "cssCommAlgorithm.h"

%extend CssComm {
%pythoncode %{
    @property
    def chebyPolynomials(self):
        return self.getChebyPolynomials()

    @chebyPolynomials.setter
    def chebyPolynomials(self, value):
        self.setChebyPolynomials(value)
%}
}

%include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
