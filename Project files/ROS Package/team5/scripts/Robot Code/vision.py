import cv2
from std_msgs.msg import Int32
import rospy



detect=0   
dummy=1  
face=0  

def visioncallback(data):
    global face
    face=data.data 


if __name__ == "__main__":
    
    
    rospy.init_node("vision")
    pub = rospy.Publisher("/visionoutput", Int32, queue_size=1)
    rospy.Subscriber("/startvision", Int32, visioncallback)
    
    
    face_cascade = cv2.CascadeClassifier('intro_to_cv/haar/haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)
    while cap.isOpened() :
        
        ret, frame = cap.read()
        cv2.imshow("camera's frame", frame)
        #face_detected_in_this_frame = False
        
        if(ret):
            
            if(face==1):
                #print("vision started")
                dummy=0
                image_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
                scale_factor = 1.3
                min_neighbours = 5
                faces = face_cascade.detectMultiScale(image_gray, scale_factor, min_neighbours)
                for (x,y,w,h) in faces:
                    cv2.rectangle(image_gray, (x,y), (x+w,y+h), (255, 255, 255) ,2) 
                    #face_detected_in_this_frame = True
                    detect=1
                cv2.imshow('frame', image_gray)
               
    
            elif face==0 and dummy==0:
                print("output sent")
                
                pub.publish(detect)
                print(detect)
                detect=0
                dummy=1

            k = cv2.waitKey(1)
            if k == 27:
                
                cv2.destroyAllWindows()
                break
