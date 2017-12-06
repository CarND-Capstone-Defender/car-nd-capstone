*********************************************
1. 	download testdata-archive from
	https://mega.nz/#F!ldJhzRhL!NWASXMs4cWegrYYNbJ7bEg
*********************************************

*********************************************
2. 	Unzip it into a folder e.g. /tmp
*********************************************


*********************************************
3.	Clone https://github.com/davisking/dlib
*********************************************


*********************************************
4.	Go inside tools/imglab and follow the readme to build imglab

	    cd dlib/tools/imglab
	    mkdir build
	    cd build
	    cmake ..
	    cmake --build . --config Release

		"You will use the imglab tool to label these objects.  
		Do this by typing the following command:
		    ./imglab -c udacity_real_dataset.xml /tmp/testdata
		
		This will create a file called udacity_real_dataset.xml which simply lists the images in /tmp/images.  

		To add bounding boxes to the objects you run:
		    ./imglab udacity_real_dataset.xml
		and a window will appear showing all the images. "

		* labels are either "Red", "Yellow" or "Green"
		* bounding boxes can be created by pressing Shift+Mouse

*********************************************

*********************************************
5. Allowed labels are Red, Yellow, Green
*********************************************


*********************************************
6. Example of a file udacity_real_dataset.xml with 1 label entry

<?xml version='1.0' encoding='ISO-8859-1'?>
<?xml-stylesheet type='text/xsl' href='image_metadata_stylesheet.xsl'?>
<dataset>
<name>imglab dataset</name>
<comment>Created by imglab tool.</comment>
<images>
  <image file='/media/sf_Term3/Final_Project_Carla/CarND-Capstone-Project/testdata_1/just_traffic_light0150.jpg'>
    <box top='373' left='471' width='46' height='132'>
      <label>Red</label>
    </box>
  </image>
</images> 	

*********************************************
