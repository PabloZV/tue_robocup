/**
 * USB ON UBUNTU SERVER EDITION (AMIGO)
 * 
 * Before starting this node, make sure you type:
 * $ pumount --yes-I-really-want-lazy-unmount /media/usb-stick/
 * $ pmount /dev/sdc1 usb-stick
 * 
 * This will mount the device at /media/sdc1 (which is now hardcoded as path) with the correct permissions
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#include "hpdf.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "challenge_emergency/Start.h"

//! Dealing with files
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <dirent.h>

#ifndef HPDF_NOPNGLIB
jmp_buf env;

#ifdef HPDF_DLL
void  __stdcall
#else
void
#endif
error_handler  (HPDF_STATUS   error_no,
                HPDF_STATUS   detail_no,
                void         *user_data)
{
    printf ("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
            (HPDF_UINT)detail_no);
    longjmp(env, 1);
}


using namespace std;
ros::ServiceServer startupSrv_;

const double scaledWidth = 1/0.025;
const double scaledHeight = 1/0.025;
const int originOffsetX = 41;
const int originOffsetY = 100;

int createPDF()
{
    ros::NodeHandle nh("~");
    //! Define string to pull out each line
    string STRING;
    ifstream myfile;
    int number_lines = 0;

    //! Find number of lines
    myfile.open((ros::package::getPath("challenge_emergency")+"/output/status.txt").c_str());
    while(getline(myfile,STRING))
    {
        number_lines++;
    }
    myfile.close();

    //! Store number of peoples and fires for header
    int number_fires = 0; int number_people = 0; int number_wounded = 0;

    //! Reopen to fill arrays status and coordinates
    int status[number_lines];
    float coordinates[number_lines][2];
    myfile.open((ros::package::getPath("challenge_emergency")+"/output/status.txt").c_str());



    //! Actual filling
    for (int ii = 0; ii < number_lines; ii++)
    {

        //! Go through every line
        getline(myfile,STRING);
        unsigned found = STRING.find(";");  //find first ';'
        STRING = STRING.substr(found+1);
        //! Prutscode to seperate 'status'/'coordinates'
        string statuss = STRING.substr(0,1);//status = {0,1,2}

        found = STRING.find(";");
        STRING = STRING.substr(found+1);    //move to next ';'
        found = STRING.find(";");
        string x = STRING.substr(0,found);  //length unknown (float)
        string y = STRING.substr(found+1);  //length unknown (float)

        //! Save 'status' information to array
        status[ii] = atoi(statuss.c_str());
        if (status[ii] == 2)
        {
            number_fires++;
        }
        else if (status[ii]==0)
        {
            number_wounded++;
            number_people++;
        }
        else
        {
            number_people++;
        }
        //! Save coordinates of object (fire/person)  in 'x' and 'y'
        coordinates[ii][0] = atof(x.c_str());
        coordinates[ii][1] = atof(y.c_str());
    }
    myfile.close();

    HPDF_Doc  pdf;
    HPDF_Font font;
    HPDF_Page page[5];
    const char* fname;
    HPDF_Destination dst;
    HPDF_Image image_map;
    HPDF_Image image_fire;
    HPDF_Image image_person;
    HPDF_Image image_symbolic_fire;



    //! Position of map
    double x_map;
    double y_map;

    //! Number of page
    int n_page = 0;

    //! Image width and height
    double iw;
    double ih;

    //! Find USB name
    bool found_file_in_media = false;
    DIR *dir;
    struct dirent *ent;
    string usb_dir, img_path;
    if ((dir = opendir ("/media/")) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            ROS_INFO("file or directory in /media/: %s", string(ent->d_name).c_str());
            if (false) // TODO: hack for ubuntu server strlen(ent->d_name)>3)
            {
                printf("%s\n", ent->d_name);
                usb_dir = "/media/" + string(ent->d_name) + "/emergency_paper.pdf";
                img_path = "/media/" + string(ent->d_name) + "/";
                found_file_in_media = true;

            }
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        return EXIT_FAILURE;
    }
    
    if (!found_file_in_media) {
		ROS_WARN("No files/directories found in /media/ -> no USB stick to write to");
		usb_dir = "/media/usb-stick/emergency_paper.pdf";
		//usb_dir = ros::package::getPath("challenge_emergency")+"/emergency_paper.pdf";
		img_path = "/media/usb-stick/";
		//img_path = ros::package::getPath("challenge_emergency")+"/";
	}

    cout << usb_dir << endl;

    fname = usb_dir.c_str();
    // save location
    ROS_INFO("usb_dir = %s", usb_dir.c_str());
    ROS_INFO("pdf will be stored as %s", fname);
    //strcpy (fname, "/media/2856-DCA4/emergency_paper");
    //strcat (fname, ".pdf");

    pdf = HPDF_New (error_handler, NULL);
    if (!pdf) {
        printf ("error: cannot create PdfDoc object\n");
        return 1;
    }

    //! Error Handler
    if (setjmp(env)) {
        HPDF_Free (pdf);
        return 1;
    }

    HPDF_SetCompressionMode (pdf, HPDF_COMP_ALL);

    //! Create default font
    font = HPDF_GetFont (pdf, "Helvetica", NULL);

    //! Add a new page object
    page[n_page] = HPDF_AddPage (pdf);

    HPDF_Page_SetWidth (page[n_page], 550);
    HPDF_Page_SetHeight (page[n_page], 650);

    //! Position on page
    double x = 50;
    double y = HPDF_Page_GetHeight (page[n_page]) - 50; // 0;

    dst = HPDF_Page_CreateDestination (page[n_page]);
    HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
    HPDF_SetOpenAction(pdf, dst);

    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
    HPDF_Page_EndText (page[n_page]);

    //! Create header with overview of information
    y = y - 40;
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 16);
    HPDF_Page_MoveTextPos (page[n_page], x, y); // HEADER
    HPDF_Page_ShowText (page[n_page], "Header");
    HPDF_Page_EndText (page[n_page]);

    y = y-18;
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], "Location: Kitchen room");
    HPDF_Page_EndText (page[n_page]);

    y = y -14;
    stringstream ss_number_fires;
    ss_number_fires<< "Number of fires: " << number_fires << ".";
    // SS to char
    std::string s = ss_number_fires.str();
    const char* string_number_fires = s.c_str();
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], string_number_fires);
    HPDF_Page_EndText (page[n_page]);

    y = y -14;
    stringstream ss_number_people;
    ss_number_people<< "Number of people: " << number_people <<", require assistance: " << number_wounded << ".";
    // SS to char
    std::string s2 = ss_number_people.str();
    const char* string_number_people = s2.c_str();
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], string_number_people);
    HPDF_Page_EndText (page[n_page]);

    y = y -14;
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], "If text is red: people need help");
    HPDF_Page_EndText (page[n_page]);

    y = y - 14;

    //! Load image of 'map' and 'fire' and 'symbolic fire'
    image_map = HPDF_LoadPngImageFromFile (pdf, (ros::package::getPath("challenge_emergency")+"/output/map.png").c_str());
    image_fire = HPDF_LoadPngImageFromFile (pdf, (ros::package::getPath("challenge_emergency")+"/output/fire.png").c_str());
    image_symbolic_fire = HPDF_LoadPngImageFromFile (pdf, (ros::package::getPath("challenge_emergency")+"/output/fire-graphic.png").c_str());


    //! Image width and height of map needed for transformations of placing fire and people in map
    iw = HPDF_Image_GetWidth (image_map);
    if (iw > 500)
    {
        iw = 500;//may not exceed page width
    }
    ih = HPDF_Image_GetHeight (image_map);
    HPDF_Page_SetLineWidth (page[n_page], 0.5);

    //! Coordinates for map (needed for placing persons/fire in map)
    //y = y - (ih+50);
    y_map = y;

    // Need to be given
    double x_null;// = (450/2)-x_map;
    double y_null;// = y_map;

    //get namespace
    string ns = ros::this_node::getName();
    ROS_INFO("ns = %s",ns.c_str());
    //! Starting location of the map
    if (nh.getParam(ns+"/x_null", x_null))
    {
        ROS_INFO("Got param x_null: %f", x_null);
    }
    else
    {
        ROS_ERROR("Failed to get param 'x_null'.");
    }
    if (nh.getParam(ns+"/y_null", y_null))
    {
        ROS_INFO("Got param y_null: %f", y_null);
    }
    else
    {
        ROS_ERROR("Failed to get param 'x_null'.");
    }


    double length_map;
    double height_map;
    //! Scaling between image and actual  map
    if (nh.getParam(ns+"/length_map", length_map))
    {
        ROS_INFO("Got param length_map: %f", length_map);
    }
    else
    {
        ROS_ERROR("Failed to get param 'length_map'.");
    }
    if (nh.getParam(ns+"/height_map", height_map))
    {
        ROS_INFO("Got param height_map: %f", height_map);
    }
    else
    {
        ROS_ERROR("Failed to get param 'height_mapl'.");
    }


    //! Read map into OpenCV
    cv::Mat image_map_cv = cv::imread(ros::package::getPath("challenge_emergency") + "/output/map.png");
    ROS_INFO("Loaded map image");

    //! Calculate tranformation ratio between pixels/meters
    double ratios [2] = {iw/length_map, ih/height_map};
    cout<<ratios[0]<<ratios[1]<<endl;
    //double ratios [2] = {3,3};

    //! Initialize strings to display person (number, status, coordinates and image)
    char person_num[20];
    char * person_stat;
    char person_coords[50];
    char person_image[50];
    char * new_str ;
    float r = 0; //print red or black depends on status


    //! Find the number of persons (= status-'fire')
    int n_person = sizeof(status)/sizeof(int);

    //! Loop over persons and fire!
    int person_index = 1;
    ROS_INFO("Start looping over %d persons (and fire)", n_person);
    for (int i = 0; i < n_person; i++){

        //! Position person/fire
        int pos_x = coordinates[i][0] * scaledWidth+originOffsetX;
        int pos_y = image_map_cv.size().height - coordinates[i][1] * scaledHeight-originOffsetY;

        //! Give number to person
        sprintf(person_num, "Person %d, ", i);

        //! Give status to person
        if (status[i]==0)
        {
            person_stat = "\n Status: Need Assistance!";
            r = 1;


            //! Draw a picture of 'fire' with subscript
            stringstream ss;
            ss << "pers" << person_index;
            cv::putText(image_map_cv, ss.str(), cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            cv::putText(image_map_cv, "help!", cv::Point(pos_x, pos_y+20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            ++person_index;
            
            ROS_INFO("Added text to image (help)");
        }
        else if (status[i]==1)
        {
            person_stat = "\n Status: Ok!";
            r = 0;


            //! Draw a picture of 'fire' with subscript
            stringstream ss;
            ss << "pers" << person_index;
            cv::putText(image_map_cv, ss.str(), cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            cv::putText(image_map_cv, "=ok", cv::Point(pos_x, pos_y+20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            ++person_index;
            
			ROS_INFO("Added text to image (ok)");

        }


        //! 'status'==2 then 'fire' location immediatly print
        else
        {
			
			ROS_INFO("Found fire");
			
            //! Image size of fire
            iw = 150;
            ih = 150;

            //! Draw a picture of 'fire' with subscript
            string path_fire_small = ros::package::getPath("challenge_emergency") + "/output/fire_small.png";
            ROS_INFO("Reading fire image: %s", path_fire_small.c_str());
            cv::Mat img_fire = cv::imread(path_fire_small.c_str());
            ROS_INFO("Loaded image fire_small");
            int fire_width = img_fire.size().width;
            int fire_height = img_fire.size().height;
            ROS_INFO("Position fire must be (%d,%d)", pos_x, pos_y);
            ROS_INFO("Image fire has size %dx%d", fire_width, fire_height);
            ROS_INFO("Image map has size %dx%d", image_map_cv.size().width, image_map_cv.size().height);
            if (pos_x-fire_width/2+fire_width <= image_map_cv.size().width && pos_y-fire_height/2+fire_height < image_map_cv.size().height) {
                img_fire.copyTo(image_map_cv(cv::Rect(pos_x-fire_width/2, pos_y-fire_height/2, fire_width, fire_height)));
            } else {
                cv::putText(image_map_cv, "X", cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            }
            
            ROS_INFO("Added fire icon to image");


            /*
            sprintf(person_coords, "FIRE location(x,y): x = %f, y = %f", coordinates[i][0], coordinates[i][1]);
            HPDF_Page_DrawImage (page[n_page], image_fire, x, y, iw, ih);
            HPDF_Page_BeginText (page[n_page]);
            HPDF_Page_SetFontAndSize (page[n_page], font, 12);
            HPDF_Page_MoveTextPos (page[n_page], x, y-12.5);
            HPDF_Page_ShowText (page[n_page], person_coords);
            HPDF_Page_EndText (page[n_page]);
            y = y - 130;

            //! Draw the fire in the map
            //! Add symbolic fire to map at CORRECT LOCATION
            stringstream ss_symbolic_fire;
            ss_symbolic_fire << "x = " << coordinates[i][0] << ",\n y = " << coordinates[i][1];
            // SS to char
            std::string s = ss_symbolic_fire.str();
            const char* string_symbolic_fire = s.c_str();
            */

            /*
            HPDF_Page_DrawImage (page[0], image_symbolic_fire, x_map+ratios[0]*(x_null+coordinates[i][0]), y_map+ratios[1]*(y_null+coordinates[i][1]), 20, 20);
            HPDF_Page_BeginText (page[0]);
            HPDF_Page_SetFontAndSize (page[0], font, 8);
            HPDF_Page_MoveTextPos (page[0], x_map+ratios[0]*(x_null+coordinates[i][0])-5, y_map+ratios[1]*(y_null+coordinates[i][1])-10);
            HPDF_Page_ShowText (page[0], string_symbolic_fire);
            HPDF_Page_EndText (page[0]);
            */

            continue;
        }



        // y = y - ih;

        /*
            HPDF_Page_DrawImage (page[n_page], image_map, x, y, iw, ih);
            HPDF_Page_BeginText (page[n_page]);
            HPDF_Page_SetFontAndSize (page[n_page], font, 12);
            HPDF_Page_MoveTextPos (page[n_page], x, y-25.5);
            HPDF_Page_ShowText (page[n_page], "Apartment: red coordinates present person(s) in need of assistance");
            HPDF_Page_EndText (page[n_page]);
        */
        //! New page if y is below treshold

        if (y < 300)
        {
            ROS_INFO("y = %f, new page needed", y);
            n_page = n_page + 1;
            //! Add a new page object
            page[n_page] = HPDF_AddPage (pdf);

            HPDF_Page_SetWidth (page[n_page], 550);
            HPDF_Page_SetHeight (page[n_page], 650);

            dst = HPDF_Page_CreateDestination (page[n_page]);
            HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
            HPDF_SetOpenAction(pdf, dst);

            HPDF_Page_BeginText (page[n_page]);
            HPDF_Page_SetFontAndSize (page[n_page], font, 12);
            HPDF_Page_MoveTextPos (page[n_page], x, HPDF_Page_GetHeight (page[n_page]) - 50);
            HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
            HPDF_Page_EndText (page[n_page]);
            y = HPDF_Page_GetHeight (page[n_page]) - 100;
            ROS_INFO("y updated to %f", y);
        }

        //! List of people (pictures, status and coordinates)
        //! Start a list with header
        HPDF_Page_BeginText (page[n_page]);
        HPDF_Page_SetFontAndSize (page[n_page], font, 12);
        HPDF_Page_MoveTextPos (page[n_page], x, y);
        HPDF_Page_ShowText (page[n_page], "List of People:");
        HPDF_Page_EndText (page[n_page]);
        y -= 170;
        
        ROS_INFO("Print list of people text");


        //! Give location to person
        sprintf(person_coords, "\t Location(x,y): x = %f, y = %f,", coordinates[i][0], coordinates[i][1]);
        

        //! Merge person and status and location
        if((new_str = (char*) malloc(strlen(person_num)+strlen(person_stat)+strlen(person_coords))+1) != NULL){
            new_str[0] = '\0';   // ensures the memory is an empty string
            strcat(new_str,person_num);
            strcat(new_str,person_coords);
            strcat(new_str,person_stat);
        }
        else {
            printf("malloc failed!\n");
        }
        //! Load image of person
#ifndef __WIN32__
        sprintf(person_image, "/output/person_%d.png",i+1);
        string string_person_image = ros::package::getPath("challenge_emergency")+person_image;
#else
        //sprintf(person_image, "pngsuite\\person_%d.png",i);
#endif
		ROS_INFO("Trying to load %s", string_person_image.c_str());
        image_person = HPDF_LoadPngImageFromFile (pdf, string_person_image.c_str());
        ROS_INFO("Loaded image person");


        //! Print picture of person and give status in PDF format
        HPDF_Page_DrawImage (page[n_page], image_person, x, y, 100, 100);
        HPDF_Page_BeginText (page[n_page]);
        HPDF_Page_SetRGBFill (page[n_page], r, 0, 0);
        HPDF_Page_SetFontAndSize (page[n_page], font, 10);
        HPDF_Page_MoveTextPos (page[n_page], x, y-10);
        HPDF_Page_ShowText (page[n_page], new_str);
        HPDF_Page_EndText (page[n_page]);
        
        ROS_INFO("Added image person");

        //! Draw person in the map
        /*
        stringstream ss_symbolic_person;
        ss_symbolic_person << "x = " << coordinates[i][0] << ",\n y = " << coordinates[i][1];
        // SS to char
        std::string s = ss_symbolic_person.str();
        const char* string_symbolic_person = s.c_str();

        //sprintf(string_symbolic_person, "x = %f, y = %f", ratio*coordinates[i][0], ratio*coordinates[i][1]);
        HPDF_Page_DrawImage (page[0], image_person, x_map+ratios[0]*(x_null+coordinates[i][0]), y_map+ratios[1]*(y_null+coordinates[i][1]), 20, 20);
        HPDF_Page_BeginText (page[0]);
        HPDF_Page_SetRGBFill (page[0], r, 0, 0);
        HPDF_Page_SetFontAndSize (page[0], font, 8);
        HPDF_Page_MoveTextPos (page[0], x_map+ratios[0]*(x_null+coordinates[i][0])-5, y_map+ratios[1]*(y_null+coordinates[i][1])-10);
        HPDF_Page_ShowText (page[0], string_symbolic_person);
        HPDF_Page_EndText (page[0]);
        */

        y = y - 120;
/*
        //! New page if y is below treshold
        if (y < 100)
        {
            n_page = n_page + 1;

            //! Add a new page object
            page[n_page] = HPDF_AddPage (pdf);

            HPDF_Page_SetWidth (page[n_page], 550);
            HPDF_Page_SetHeight (page[n_page], 650);

            dst = HPDF_Page_CreateDestination (page[n_page]);
            HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
            HPDF_SetOpenAction(pdf, dst);

            HPDF_Page_BeginText (page[n_page]);
            HPDF_Page_SetFontAndSize (page[n_page], font, 12);
            HPDF_Page_MoveTextPos (page[n_page], x, HPDF_Page_GetHeight (page[n_page]) - 50);
            HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
            HPDF_Page_EndText (page[n_page]);
            y = HPDF_Page_GetHeight (page[n_page]) - 170;
        }*/
    }

    //cv::imshow("nice_map", image_map_cv);
    //cv::waitKey(0);

    //! Store generated image on disk
    ROS_INFO("Storing image on disk...");
    string file_name_map = img_path + "img_map_cv.png";
    ROS_INFO("Image name: %s", file_name_map.c_str());
    cv::imwrite(file_name_map.c_str(), image_map_cv);
    ROS_INFO("Image stored");

    //! Load this image and draw it in pdf
    HPDF_Image image_map_generated = HPDF_LoadPngImageFromFile (pdf, file_name_map.c_str());
    //HPDF_Page_DrawImage (page[n_page], image_map, x, y, image_map_cv.size().width, image_map_cv.size().height);
    int width_map_in_pdf = image_map_cv.size().width;
    int height_map_in_pdf = image_map_cv.size().height;
    if (width_map_in_pdf > 500) {
        height_map_in_pdf = 500.0/(double)width_map_in_pdf * height_map_in_pdf;
        width_map_in_pdf = 500;
    }

    //! New page
    n_page = n_page + 1;

    //! Add a new page object
    page[n_page] = HPDF_AddPage (pdf);

    HPDF_Page_SetWidth (page[n_page], 550);
    HPDF_Page_SetHeight (page[n_page], 650);

    dst = HPDF_Page_CreateDestination (page[n_page]);
    HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
    HPDF_SetOpenAction(pdf, dst);

    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, HPDF_Page_GetHeight (page[n_page]) - 50);
    HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
    HPDF_Page_EndText (page[n_page]);
    //y = HPDF_Page_GetHeight (page[n_page]) - 170;

    //! Map in pdf
    ROS_INFO("Adding map to pdf...");
    y -= 180;
    HPDF_Page_DrawImage (page[n_page], image_map_generated, x, y, width_map_in_pdf, height_map_in_pdf);
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y-25.5);
    //HPDF_Page_ShowText (page[n_page], "Apartment: red coordinates present person(s) in need of assistance");
    HPDF_Page_EndText (page[n_page]);
    ROS_INFO("added");

    //! Save the document to a file
    ROS_INFO("Saving pdf to file...");
    HPDF_SaveToFile (pdf, fname);
    ROS_INFO("Saved pdf");

    //! Clean up
    HPDF_Free (pdf);
}


bool startUp(challenge_emergency::Start::Request& req, challenge_emergency::Start::Response& resp)
{
    // Create the pdf brother G
    createPDF();
    return true;
}

int main (int argc, char **argv)
{
    //! Start up node
    ros::init(argc, argv, "emergency_paper");
    ros::NodeHandle nh("~");
    ROS_INFO("Starting pdf creation service..");
    startupSrv_ = nh.advertiseService("startup", &startUp);

    createPDF();

    while(ros::ok())
        ros::spinOnce();
    return 0;
}
#else

int main()
{
    printf("WARNING: if you want to run this demo, \n"
           "make libhpdf with HPDF_USE_PNGLIB option.\n");
    return 0;
}

#endif /* HPDF_NOPNGLIB */
