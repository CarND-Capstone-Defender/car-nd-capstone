import tensorflow as tf
import yaml
import os
import dataset_utils
import xml.etree.ElementTree

flags = tf.app.flags
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS


# lazy implementation without a file writer....
# just dump the result in the shell in a file, e.g. convert_xml_2_yaml.py > mylabel.yaml
def main(_):
    
    # converts a label file in xml format (e.g. from ImgLab - https://github.com/davisking/dlib)    
    # into a yaml file

    # hardcoded path so far....
    INPUT_XML = "udacity_real_dataset.xml"
    e = xml.etree.ElementTree.parse(INPUT_XML).getroot()

    for atype in e.findall('images/image'):
        fileName = atype.get('file')

        if (atype.find('./box') is None):
            #print('- boxes: []')
            #print('  path: ' , fileName)
            # ignore images without any boxes....just useless overhead....
            pass
        else:
            print('- boxes:')

            for box in atype.findall('./box'):
                top = float(box.get('top'))
                left = float(box.get('left'))
                width = float(box.get('width'))
                height = float(box.get('height'))
                label = box.find('./label')
                if (label is not None):
                    print('  - {label: ' , label.text , ', occluded: false, x_max:', left+width , ' , x_min:' , left  , ' , y_max:' , top + height , ' , y_min:' , top , '}')
        
            print('  path: ' , fileName)




if __name__ == '__main__':
    tf.app.run()