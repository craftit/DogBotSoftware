

#include "Ravl/Option.hh"
#include "Ravl/ParseCSV.hh"
#include "Ravl/PatternRec/DataSetVectorLabel.hh"
#include "Ravl/PatternRec/DesignFuncLDA.hh"
#include "Ravl/PatternRec/DesignFuncLSQ.hh"


int main(int nargs,char **argv)
{
  RavlN::ParseCSVC csvFile;

  if(!csvFile.Open("cal1.csv")) {
    return false;
  }
  RavlN::SArray1dC<RavlN::StringC> values;

  RavlN::OStreamC fout("data.csv");
  RavlN::SampleC<RavlN::VectorC> in;
  RavlN::SampleC<RavlN::VectorC> out;

  while(true) {
    uint16_t sensors[3];

    if(!csvFile.ReadValues(values))
      break;
    uint16_t sensorsIn[3];
    sensorsIn[0] = values[1].IntValue();
    sensorsIn[1] = values[2].IntValue();
    sensorsIn[2] = values[3].IntValue();



    int rval = values[0].IntValue();
    int target = (rval%12);

    float angle = target * M_PI * 2.0 / 12.0;

    RavlN::VectorC vin(3);
    vin[0] = sensorsIn[0];
    vin[1] = sensorsIn[1];
    vin[2] = sensorsIn[2];
    //vin.MakeUnit();

    RavlN::VectorC vout(2);
    vout[0] = sin(angle);
    vout[1] = cos(angle);

    in.Append(vin);
    out.Append(vout);
    fout << sensorsIn[0] << "," << sensorsIn[1] << "," << sensorsIn[0] << "," << target << "\n";

  }

  RavlN::DesignFuncLSQC lsq(1,false);
  RavlN::FunctionC func = lsq.Apply(in,out);
  RavlDebug("Func:%s",RavlN::StringOf(func).c_str());

}
