import rospy
from qr_navigation.srv import set_target, set_targetRequest
from tkinter import *
from inspect import *
from tkinter import ttk

def client(latitud,longitud,message):
    print(message)
    result=set_target_function(latitud,longitud,message)
    print(result.latitud2)
    print(result.longitud2)
    print(result.mode2)

if __name__ == '__main__':
    try:
        set_target_function = rospy.ServiceProxy('set_target', set_target)
        window = Tk()
        window.geometry("500x500")
        window.configure(bg="black")
        Title = Label(window,width=20,height=3,text="Coordinate Publisher",fg="blue",bg="white",font=("TimesNewRoman",15))
        Title.place(x=140,y=0)
        LatitudLabel = Label(window,width=20,height=3,text="Latitud",background="black",fg="white")
        LatitudLabel.place(x=0,y=100)
        latitud = Text(width=20,height=4,bg="white")
        latitud.place(x=150,y=100)
        LongitudLabel = Label(window,width=20,height=3,text="Longitud",background="black",fg="white")
        LongitudLabel.place(x=0,y=200)
        longitud = Text(width=20,height=4,bg="white")
        longitud.place(x=150,y=200)
        combo = ttk.Combobox(state="readonly",values=["gps_and_post","gps_and_gate","gps_only"])
        combo.place(x=150,y=300)
        publish = Button(window,width=10,height=4,text="publish",fg="blue",bg="black",command=lambda m="F":client(float(latitud.get("1.0","end-1c")),float(longitud.get("1.0","end-1c")),combo.get()))
        publish.place(x=180,y=350)
        window.mainloop()
    except rospy.ROSInterruptException:
        pass

