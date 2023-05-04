import rospy
from coords.srv import set_target, set_targetRequest
from tkinter import *
from inspect import *

def client(latitud,longitud):
    set_target_function = rospy.ServiceProxy('get_target', set_target)
    result=set_target_function(latitud,longitud)
    print(result.latitud_r)
    print(result.longitud_r)

if __name__ == '__main__':
    try:
        window = Tk()
        window.geometry("500x500")
        window.configure(bg="black")
        Title = Label(window,width=20,height=3,text="Coordinate Publisher",fg="blue",bg="white",font=("TimesNEwRoman",15))
        Title.place(x=150,y=0)
        LatitudLabel = Label(window,width=20,height=3,text="Latitud",background="black",fg="white")
        LatitudLabel.place(x=0,y=100)
        latitud = Text(width=20,height=4,bg="white")
        latitud.place(x=150,y=100)
        LongitudLabel = Label(window,width=20,height=3,text="Latitud",background="black",fg="white")
        LongitudLabel.place(x=0,y=200)
        longitud = Text(width=20,height=4,bg="white")
        longitud.place(x=150,y=200)
        publish = Button(window,width=10,height=4,text="publish",fg="blue",bg="black",command=lambda m="F":client(float(latitud.get("1.0","end-1c")),float(longitud.get("1.0","end-1c"))))
        publish.place(x=180,y=350)
        window.mainloop()
    except rospy.ROSInterruptException:
        pass
