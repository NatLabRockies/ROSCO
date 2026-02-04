import os
from wisdem.inputs.validation import _validate
from openfast_io.FileTools import remove_numpy
from windIO.yaml import write_yaml


schema_dir = os.path.dirname(os.path.abspath(__file__))

def load_rosco_yaml(finput, rank_0=False):
    rosco_schema = os.path.join(schema_dir,'toolbox_schema.yaml')
    return _validate(finput, rosco_schema, defaults=True, rank_0=rank_0)

def write_rosco_yaml(instance, foutput):
    rosco_schema = os.path.join(schema_dir,'toolbox_schema.yaml')
    _validate(instance, rosco_schema, restrictive=False, removal=True, defaults=False, rank_0=True)
    instance2 = remove_numpy(instance)
    write_yaml(instance2, foutput)


if __name__=='__main__':
    fname = '/Users/dzalkind/Tools/ROSCO-PRC/Examples/Tune_Cases/NREL5MW.yaml'
    new_input = load_rosco_yaml(fname)

    write_rosco_yaml(new_input, 'test_output.yaml')
    
    print('here')

