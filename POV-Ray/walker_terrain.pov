#include "colors.inc"

#macro QToMatrix(Q)
  // Convert a quaternion to a Povray transformation matrix (4x3)
  // Use: matrix <M[0].x,M[0].y,M[0].z,M[1].x,M[1].y,M[1].z,M[2].x,M[2].y,M[2].z,M[3].x,M[3].y,M[3].z>
  #local X2 = Q.x + Q.x;
  #local Y2 = Q.y + Q.y;
  #local Z2 = Q.z + Q.z;
  #local XX = Q.x * X2;
  #local XY = Q.x * Y2;
  #local XZ = Q.x * Z2;
  #local YY = Q.y * Y2;
  #local YZ = Q.y * Z2;
  #local ZZ = Q.z * Z2;
  #local TX = Q.t * X2;
  #local TY = Q.t * Y2;
  #local TZ = Q.t * Z2;
  array[4] {<1.0 - (YY + ZZ),XY + TZ,XZ - TY>,<XY - TZ,1.0 - (XX + ZZ),YZ + TX>,<XZ + TY,YZ - TX,1.0 - (XX + YY)>,<0,0,0>}
#end

#macro QMatrix(Q)
  // Use quaternion directly as an object modifier
  #local M = QToMatrix(Q)
  transform { matrix <M[0].x,M[0].y,M[0].z,M[1].x,M[1].y,M[1].z,M[2].x,M[2].y,M[2].z,M[3].x,M[3].y,M[3].z> }
#end

#macro QToEuler(Q)
  // Quaternion to Euler angles
  // FAILS with difficult situations (Euler problems), this needs work to avoid errors...
  #local Q2 = Q*Q;
  <atan2 (2*(Q.y*Q.z+Q.x*Q.t), (-Q2.x-Q2.y+Q2.z+Q2.t)),
   asin (-2*(Q.x*Q.z-Q.y*Q.t)),
   atan2 (2*(Q.x*Q.y+Q.z*Q.t), (Q2.x-Q2.y-Q2.z+Q2.t))>
#end

#declare fnum=abs(frame_number);
#declare morelight=false;
#declare rad=true;

#if(rad)
// radiosity (global illumination) settings
global_settings {
 max_trace_level 3   
 
 radiosity {
    pretrace_start 0.08
     pretrace_end   0.01
     count 50
     nearest_count 10
     error_bound 0.15
     recursion_limit 1
     low_error_factor 0.2
     gray_threshold 0
     minimum_reuse 0.015
     brightness 1.0
     adc_bailout 0.01
  }  
  
}

#end

// set a color of the background (sky)
//background { color rgb <1, 1, 1> }

sky_sphere{
 pigment{ gradient <0,1,0>
         color_map{
         [0.0 color rgb<1,1,1>        ]
         [0.8 color rgb<0.5,0.5,0.5>]
         [1.0 color rgb<0.3,0.3,0.3>]}
       } // end pigment
 }





// create a regular point light source
light_source {
 1500*x                  // light's position (translated below)
 color rgb <1,1,1>    // light's color
 translate <10, 1500, 10>
}



#declare reactor_data_file1 = concat("data/pos", str(fnum,-4,0), ".txt")  
//#declare reactor_data_file1 = concat("data/pos0091.txt")  

#warning concat("---- LOADING DATA FILE : ",  reactor_data_file1, "\n")
#fopen MyPosFile1 reactor_data_file1 read
#declare ax=0.0;
#declare ay=0.0;
#declare az=0.0;
#declare e0=0.0;
#declare e1=0.0;
#declare e2=0.0;
#declare e3=0.0;
#declare rx=0.0;
#declare ry=0.0;
#declare rz=0.0;
#declare caz=0.0;
#declare arad=.1;
#declare id=0;
#declare batch=0;
                                                         


/* // walls
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <1.0,3.0,30.0>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
  pigment {color rgbt <0, 100/255, 225/255,0> }finish {diffuse 1 ambient 0.0 specular .05 }   }
  
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <1.0,3.0,30.0>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
  pigment {color rgbt <0, 100/255, 225/255,0> }finish {diffuse 1 ambient 0.0 specular .05 }   }
  
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <15.0,3.0,1.0>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
  pigment {color rgbt <0, 100/255, 225/255,0> }finish {diffuse 1 ambient 0.0 specular .05 }   }
  
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <15.0,3.0,1.0>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
  pigment {color rgbt <0, 100/255, 225/255,0> }finish {diffuse 1 ambient 0.0 specular .05 }   }
*/




              
              
// Chassis
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <2.0,1.0,15.0>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
   pigment { BrightGold }
     finish {
        ambient .1
        diffuse .1
        specular 1
        roughness .001
        metallic
        reflection {
          .75
          metallic
        }
     } 
   }
  #declare id=id+1;


// perspective (default, not required) camera
camera {
   perspective
   location <-cos(-fnum/200-1)*(40), 10, -sin(fnum/200+1)*(40)+az>
   look_at <0, 15-1500/100,  az>
   right     x*image_width/image_height  // aspect
   // direction z                        // direction and zoom
   // angle 67                           // field (overides direction zoom)
} 
 
 
 
// axles 
#while (defined(MyPosFile1)&id<4)
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  cylinder {<0,-1.5,0>,<0,1.5,0>,0.5   
  rotate<0,0,90>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az>   
  pigment { BrightGold }
     finish {
        ambient .1
        diffuse .1
        specular 1
        roughness .001
        metallic
        reflection {
          .75
          metallic
        }
     }
  }
  #declare id=id+1;
#end                            

// legs
#while (defined(MyPosFile1)&id<10)
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <1.0,2.0,0.5>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
  pigment { BrightGold }
     finish {
        ambient .1
        diffuse .1
        specular 1
        roughness .001
        metallic
        reflection {
          .75
          metallic
        }
     } 
   }
  #declare id=id+1;
#end         

// feet
#while (defined(MyPosFile1)&id<16)
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  box {<-0.5,-0.5,-0.5>,<0.5,0.5,0.5> scale <1.5,0.5,2.0>
  QMatrix(<e1,e2,e3,e0>) translate<ax, ay, az >  
  pigment { BrightGold }
     finish {
        ambient .1
        diffuse .1
        specular 1
        roughness .001
        metallic
        reflection {
          .75
          metallic
        }
     } 
  }
  #declare id=id+1;
#end          

// terrain
#while (defined(MyPosFile1))
  #read (MyPosFile1, ax, ay, az, e0,e1,e2, e3)
  sphere {<ax, ay, az >, 0.2 
  pigment {Silver} finish {reflection {0.5 metallic}}   }
  #declare id=id+1;
#end


//} 

plane {y, -7+1.5 pigment { DarkSlateGray }finish {diffuse 0.5 ambient 0.2 reflection 
{0.2, 1.0 fresnel on}}hollow on}
