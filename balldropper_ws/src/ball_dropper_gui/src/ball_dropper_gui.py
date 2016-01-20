#!/usr/bin/python

from Tkinter import *
from ttk import *
import sys
import signal
import Queue

from ball_dropper.srv import *
from ball_dropper_msgs.msg import Heartbeat
import rospy


#Class taken from: http://tkinter.unpythonic.net/wiki/VerticalScrolledFrame
class VerticalScrolledFrame(Frame):
    """A pure Tkinter scrollable frame that actually works!

    * Use the 'interior' attribute to place widgets inside the scrollable frame
    * Construct and pack/place/grid normally
    * This frame only allows vertical scrolling
    
    """
    def __init__(self, parent, *args, **kw):
        Frame.__init__(self, parent, *args, **kw)           

        # create a canvas object and a vertical scrollbar for scrolling it
        vscrollbar = Scrollbar(self, orient=VERTICAL)
        vscrollbar.pack(fill=Y, side=RIGHT, expand=FALSE)
        canvas = Canvas(self, bd=0, highlightthickness=0,
                        yscrollcommand=vscrollbar.set)
        canvas.pack(side=LEFT, fill=BOTH, expand=TRUE)
        vscrollbar.config(command=canvas.yview)

        # reset the view
        canvas.xview_moveto(0)
        canvas.yview_moveto(0)

        # create a frame inside the canvas which will be scrolled with it
        self.interior = interior = Frame(canvas)
        interior_id = canvas.create_window(0, 0, window=interior,
                                           anchor=NW)

        # track changes to the canvas and frame width and sync them,
        # also updating the scrollbar
        def _configure_interior(event):
            # update the scrollbars to match the size of the inner frame
            size = (interior.winfo_reqwidth(), interior.winfo_reqheight())
            canvas.config(scrollregion="0 0 %s %s" % size)
            if interior.winfo_reqwidth() != canvas.winfo_width():
                # update the canvas's width to fit the inner frame
                canvas.config(width=interior.winfo_reqwidth())
        interior.bind('<Configure>', _configure_interior)

        def _configure_canvas(event):
            if interior.winfo_reqwidth() != canvas.winfo_width():
                # update the inner frame's width to fill the canvas
                canvas.itemconfigure(interior_id, width=canvas.winfo_width())
        canvas.bind('<Configure>', _configure_canvas)

        return

def operation_client(op):
     rospy.wait_for_service('operation')
     try:
         operation = rospy.ServiceProxy('operation', Operation)
         resp1 = operation(op)
         return resp1.errorMessage
     except rospy.ServiceException, e:
         print "Service call failed: %s"%e

class BallDropperGUI(Frame):

	def __init__(self, parent):
		Frame.__init__(self, parent)
		self.parent = parent
		self.initUI()

		rospy.init_node('ballDropperGUI')
		rospy.Timer(rospy.Duration(0.05), self.op_timer_callback)
		rospy.Subscriber("heartbeat", Heartbeat, self.handleHeartbeat, queue_size=1)

	def initUI(self):
		self.parent.title("Ball Dropper")
		self.style = Style()
		self.style.theme_use("default")

		self.lastHbTimestamp = 0;
		self.freq = 0
		self.filterWeight = 0.15

		self.opQueue = Queue.Queue()

		self.idle = 1

		buttonWidth = 7
		fontSize = 10
		
		self.padx = 5
		self.pady = 5

		self.remainingBalls = 25
		self.remainingInjections = 0
		self.remainingBallsStr = StringVar()
		self.remainingBallsStr.set(str(self.remainingBalls))
		self.remainingInjectionsStr = StringVar()
		self.remainingInjectionsStr.set(str(self.remainingInjections))

		self.pack(fill=BOTH, expand=1)

		self.scrollFrame = VerticalScrolledFrame(self)
		self.scrollFrame.pack(fill=BOTH, expand=1)

		self.mainPanel = Frame(self.scrollFrame.interior)
		self.mainPanel.pack(fill=BOTH, expand=1)

#Primary Button Frame
		self.col1 = Frame(self.mainPanel)
		self.col1.grid(row=0, column=0, padx=self.padx, pady=self.pady, sticky='N')

		self.operationFrame = Frame(self.col1, borderwidth=1, relief=SUNKEN)
		self.operationFrame.pack(fill=BOTH, side=TOP, padx=5, pady=5)
		row=0
		width = 15
		labWidth = 12
		padx = 5
		pady = 5

		vcmdDropNum = (self.register(self.onDropNumValidate), '%P', '%S')
		self.dropButtonLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Drop:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.dropNum = Entry(self.operationFrame, width=buttonWidth, validate='key', validatecommand=vcmdDropNum)
		self.dropNum.grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.dropNum.insert(0, '0')
		self.dropButton = Button(self.operationFrame, text="Execute", command=self.drop, width=buttonWidth)
		self.dropButton.grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		vcmdRotateNum = (self.register(self.onRotateNumValidate), '%P', '%S')
		self.rotateButtonLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Rotate:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.rotateNum = Entry(self.operationFrame, width=buttonWidth, validate='key', validatecommand=vcmdRotateNum)
		self.rotateNum.grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.rotateNum.insert(0, '0')
		self.rotateButton = Button(self.operationFrame, text="Execute", command=self.rotateNTimes, width=buttonWidth)
		self.rotateButton.grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		vcmdInjectNum = (self.register(self.onInjectNumValidate), '%P', '%S')
		self.injectFieldLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Inject:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.injectFieldNum = Entry(self.operationFrame, width=buttonWidth, validate='key', validatecommand=vcmdInjectNum)
		self.injectFieldNum.grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.injectFieldNum.insert(0, '0')
		self.injectFieldButton = Button(self.operationFrame, text="Execute", command=self.injectNTimes, width=buttonWidth)
		self.injectFieldButton.grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		self.hatchButtonLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Hatch:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.hatchOpenButton = Button(self.operationFrame, text="Open", command=self.openHatch, width=buttonWidth).grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.hatchCloseButton = Button(self.operationFrame, text="Close", command=self.closeHatch, width=buttonWidth).grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		self.driveButtonLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Drive:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.driveOnButton = Button(self.operationFrame, text="On", command=self.driveOn, width=buttonWidth).grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.driveOffButton = Button(self.operationFrame, text="Off", command=self.driveOff, width=buttonWidth).grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		vcmdReloadBallsNum = (self.register(self.onReloadBallsNumValidate), '%P', '%S')
		self.reloadBallsButtonLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Reload Balls:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.reloadBallsNum = Entry(self.operationFrame, width=buttonWidth, validate='key', validatecommand=vcmdReloadBallsNum)
		self.reloadBallsNum.grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.reloadBallsNum.insert(0, '0')
		self.reloadBallsButton = Button(self.operationFrame, text="Add", command=self.reloadBalls, width=buttonWidth)
		self.reloadBallsButton.grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		vcmdReloadInjectionsNum = (self.register(self.onReloadInjectionsNumValidate), '%P', '%S')
		self.reloadInjectionsButtonLabel = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Reload Inject:", width=labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.reloadInjectionsNum = Entry(self.operationFrame, width=buttonWidth, validate='key', validatecommand=vcmdReloadInjectionsNum)
		self.reloadInjectionsNum.grid(row=row, column=1, padx=padx, pady=pady, sticky="W")
		self.reloadInjectionsNum.insert(0, '30')
		self.reloadInjectionsNum.config(state='readonly')
		self.reloadInjectionsButton = Button(self.operationFrame, text="Set", command=self.reloadInject, width=buttonWidth)
		self.reloadInjectionsButton.grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

		self.placeholder = Label(self.operationFrame, font=("TkDefaultFont", fontSize), text="Placeholder").grid(row=row, column=0, padx=self.padx, pady=self.pady, sticky="W")
		self.placeholderButton = Button(self.operationFrame, text="Do", width=buttonWidth).grid(row=row, column=2, padx=padx, pady=pady, sticky="W")
		row += 1

#Secondary Button Frame
		self.col2 = Frame(self.mainPanel)
		self.col2.grid(row=0, column=1, padx=self.padx, pady=self.pady, sticky='N')

		#Counters
		self.counterFrame = Frame(self.col2, borderwidth=1, relief=SUNKEN)
		self.counterFrame.pack(fill=BOTH, side=TOP, padx=5, pady=5)
		row=0
		width = 15
		padx = 5
		pady = 5

		self.counterLabel = Label(self.counterFrame, font=("TkDefaultFont", fontSize), text="Counter Operations:").grid(row=row, column=0, padx=padx, pady=pady, sticky="W", columnspan=2)
		row += 1

		self.dCounter = IntVar()
		self.dCounterButton = Button(self.counterFrame, text="Driver", command=self.setDCounter, width=buttonWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.dCounterVal = Entry(self.counterFrame, textvariable=self.dCounter, width=width).grid(row=row, column=1, padx=padx, pady=pady, sticky="E", columnspan=2)
		row += 1

		self.rCounter = IntVar()
		self.dCounterButton = Button(self.counterFrame, text="Rotator", command=self.setRCounter, width=buttonWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.dCounterVal = Entry(self.counterFrame, textvariable=self.rCounter, width=width).grid(row=row, column=1, padx=padx, pady=pady, sticky="E", columnspan=2)
		row += 1

		self.iCounter = IntVar()
		self.iCounterButton = Button(self.counterFrame, text="Injector", command=self.setICounter, width=buttonWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.iCounterVal = Entry(self.counterFrame, textvariable=self.iCounter, width=width).grid(row=row, column=1, padx=padx, pady=pady, sticky="E", columnspan=2)
		row += 1

		#Thresholds
		self.threshFrame = Frame(self.col2, borderwidth=1, relief=SUNKEN)
		self.threshFrame.pack(fill=BOTH, side=TOP, padx=5, pady=5)
		row=0

		self.threshLabel = Label(self.threshFrame, font=("TkDefaultFont", fontSize), text="Threshold Operations:").grid(row=row, column=0, padx=padx, pady=pady, sticky="W", columnspan=2)
		row += 1

		self.dLowThresh = IntVar()
		self.dHighThresh = IntVar()
		self.dThreshButton = Button(self.threshFrame, text="Driver", command=self.setDThresh, width=buttonWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.dLowThreshVal = Entry(self.threshFrame, textvariable=self.dLowThresh, width=width/2).grid(row=row, column=1, padx=(padx, 0), pady=pady, sticky="W")
		self.dHighThreshVal = Entry(self.threshFrame, textvariable=self.dHighThresh, width=width/2).grid(row=row, column=2, padx=(0, padx), pady=pady, sticky="E")
		row += 1

		self.rThresh = IntVar()
		self.rThreshButton = Button(self.threshFrame, text="Rotator", command=self.setRThresh, width=buttonWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.rThreshVal = Entry(self.threshFrame, textvariable=self.rThresh, width=width).grid(row=row, column=1, padx=padx, pady=pady, sticky="E", columnspan=2)
		row += 1

		self.iThresh = IntVar()
		self.iThreshButton = Button(self.threshFrame, text="Injector", command=self.setIThresh, width=buttonWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.iThreshVal = Entry(self.threshFrame, textvariable=self.iThresh, width=width).grid(row=row, column=1, padx=padx, pady=pady, sticky="E", columnspan=2)
		row += 1

#Remaining Supplies Column
		self.col4 = Frame(self.mainPanel)
		self.col4.grid(row=1, column=0, padx=self.padx, pady=self.pady, sticky='NW', columnspan=2)

#Supplies Frame
		self.suppliesFrame = Frame(self.col4, relief=SUNKEN, borderwidth=1)
		self.suppliesFrame.pack(fill=BOTH, padx=5, pady=5)
		row = 0
		padx = 5
		pady = 1
		width = 15
		labWidth = 20

		self.remainingBallsLabel = Label(self.suppliesFrame, font=("TkDefaultFont", fontSize), text="Remaining Balls:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.remainingBallsVal = Label(self.suppliesFrame, font=("TkDefaultFont", fontSize), textvariable=self.remainingBallsStr, width=width, anchor="e").grid(row=row, column=1, padx=padx, pady=pady, sticky="E")
		row += 1

		self.remainingInjectionsLabel = Label(self.suppliesFrame, font=("TkDefaultFont", fontSize), text="Remaining Injections:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.remainingInjectionsVal = Label(self.suppliesFrame, font=("TkDefaultFont", fontSize), textvariable=self.remainingInjectionsStr, width=width, anchor="e").grid(row=row, column=1, padx=padx, pady=pady, sticky="E")
		row += 1

#Warning Frame

		self.warningFrame = Frame(self.col4, relief=SUNKEN, borderwidth=1)
		self.warningFrame.pack(fill=BOTH, padx=5, pady=5)
		row = 0
		width = 15
		labWidth = 20

		self.fireWarningLabel = Label(self.warningFrame, font=("TkDefaultFont", fontSize), text="Fire Danger:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.fireWarningVal = Label(self.warningFrame, font=("TkDefaultFont", fontSize), text="", width=width, anchor="e")
		self.fireWarningVal.grid(row=row, column=1, padx=padx, pady=pady, sticky="E")
		row += 1

		self.hardwareFailure = StringVar()
		self.hardwareFailureLabel = Label(self.warningFrame, font=("TkDefaultFont", fontSize), text="Hardware Failure:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.hardwareFailureVal = Label(self.warningFrame, font=("TkDefaultFont", fontSize), textvariable=self.hardwareFailure, width=width, anchor="e")
		self.hardwareFailureVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1


#Info Column
		self.col3 = Frame(self.mainPanel)
		self.col3.grid(row=0, column=2, padx=self.padx, pady=self.pady, sticky='N', rowspan=2)

#Connectivity Frame

		self.connFrame = Frame(self.col3, relief=SUNKEN, borderwidth=1)
		self.connFrame.pack(fill=BOTH, padx=5, pady=5)
		row = 0
		width = 15
		labWidth = 20

		self.timestampLabel = Label(self.connFrame, font=("TkDefaultFont", fontSize), text="Heartbeat Timestamp:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.timestampVal = Label(self.connFrame, font=("TkDefaultFont", fontSize), text="", width=width, anchor="e")
		self.timestampVal.grid(row=row, column=1, padx=padx, pady=pady, sticky="E")
		row += 1

		self.freqLabel = Label(self.connFrame, font=("TkDefaultFont", fontSize), text="Heartbeat Frequency:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.freqVal = Label(self.connFrame, font=("TkDefaultFont", fontSize), width=width, anchor="e")
		self.freqVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.connLabel = Label(self.connFrame, font=("TkDefaultFont", fontSize), text="Connectivity:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.connVal = Label(self.connFrame, font=("TkDefaultFont", fontSize), width=width, anchor="e", text='Ok', foreground='green')
		self.connVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

#Hearbeat Frame
		self.heartbeatFrame = Frame(self.col3, relief=SUNKEN, borderwidth=1)
		self.heartbeatFrame.pack(fill=BOTH, padx=5, pady=5)
		row = 0
		padx = 5
		pady = 1
		width = 15
		labWidth = 20

		self.currOp = StringVar()
		self.currOpLabel = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="Current OP:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.currOpVal = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), textvariable=self.currOp, width=width, anchor="e")
		self.currOpVal.grid(row=row, column=1, padx=padx, pady=pady, sticky="E")
		row += 1

		self.lastOp = StringVar()
		self.lastOpLabel = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="Last OP:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.lastOpVal = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), textvariable=self.lastOp, width=width, anchor="e")
		self.lastOpVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.calibratedLabel = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="Calibrated:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.calibratedVal = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="0", width=width, anchor="e")
		self.calibratedVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.hatchStateLabel = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="Hatch:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.hatchStateVal = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="0", width=width, anchor="e")
		self.hatchStateVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.wheelStateLabel = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="Wheel:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.wheelStateVal = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="0", width=width, anchor="e")
		self.wheelStateVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.driverStateLabel = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="Driver:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.driverStateVal = Label(self.heartbeatFrame, font=("TkDefaultFont", fontSize), text="0", width=width, anchor="e")
		self.driverStateVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

#Motor Monitor Frame

		self.motorMonitorFrame = Frame(self.col3, relief=SUNKEN, borderwidth=1)
		self.motorMonitorFrame.pack(fill=BOTH, padx=5, pady=5)
		row = 0
		width = 15
		labWidth = 20

		self.currMotor = StringVar()
		self.currMotorLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Motor:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.currMotorVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.currMotor, width=width, anchor="e")
		self.currMotorVal.grid(row=row, column=1, padx=padx, pady=pady, sticky="E")
		row += 1

		self.duration = StringVar()
		self.durationLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Action Duration:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.durationVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.duration, width=width, anchor="e")
		self.durationVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1
		
		self.counterStart = IntVar()
		self.counterStartLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Counter Start:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.counterStartVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.counterStart, width=width, anchor="e")
		self.counterStartVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.counterEnd = IntVar()
		self.counterEndLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Counter Start:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.counterEndVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.counterEnd, width=width, anchor="e")
		self.counterEndVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.instCurrent = StringVar()
		self.instCurrentLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Inst. Current:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.instCurrentVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.instCurrent, width=width, anchor="e")
		self.instCurrentVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.minCurrent = StringVar()
		self.minCurrentLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Lowest Current:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.minCurrentVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.minCurrent, width=width, anchor="e")
		self.minCurrentVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.maxCurrent = StringVar()
		self.maxCurrentLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Highest Current:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.maxCurrentVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.maxCurrent, width=width, anchor="e")
		self.maxCurrentVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1

		self.totCurrent = StringVar()
		self.totCurrentLabel = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), text="Total Current:", width = labWidth).grid(row=row, column=0, padx=padx, pady=pady, sticky="W")
		self.totCurrentVal = Label(self.motorMonitorFrame, font=("TkDefaultFont", fontSize), textvariable=self.totCurrent, width=width, anchor="e")
		self.totCurrentVal.grid(row=row, column=1, padx=padx, pady=2, sticky="E")
		row += 1
		

	def onDropNumValidate(self, P, S):
		valid = (P == '') or P.isdigit()
		if P.isdigit():
			valid = (int(S) in xrange(10) and int(P) <= self.remainingBalls)
		return valid

	def onRotateNumValidate(self, P, S):
		valid = (P == '') or P.isdigit()
		if P.isdigit():
			valid = (int(S) in xrange(10) and int(P) <= 100)
		return valid

	def onInjectNumValidate(self, P, S):
		valid = (P == '') or P.isdigit()
		if P.isdigit():
			valid = (int(S) in xrange(10) and int(P) <= self.remainingInjections)
		return valid

	def onReloadBallsNumValidate(self, P, S):
		valid = (P == '') or P.isdigit()
		if P.isdigit():
			valid = (int(S) in xrange(10) and self.remainingBalls + int(P) <= 50)
		return valid

	def onReloadInjectionsNumValidate(self, P, S):
		valid = int(P) == 30
		return valid

	def drop(self):
		if self.dropNum.get().isdigit():
			for i in range(0,int(self.dropNum.get())):
				self.driveOn()
				self.inject()
				self.driveOff()
				self.rotate()
				self.remainingBalls -= 1
		if self.remainingBalls == 0:
			self.dropButton.config(state='disabled')
		if int(self.dropNum.get()) > self.remainingBalls:
			self.dropNum.delete(0, 'end')
			self.dropNum.insert(0, str(self.remainingBalls))
		
	def rotateNTimes(self):
		if self.rotateNum.get().isdigit():
			for i in range(0, int(self.rotateNum.get())):
				self.rotate()

	def injectNTimes(self):
		if self.injectFieldNum.get().isdigit():
			for i in range(0,int(self.injectFieldNum.get())):
				self.inject()
		else:
			print('Failed')
		self.updateInjectList()
		if int(self.injectFieldNum.get()) > self.remainingInjections:
			self.injectFieldNum.delete(0, 'end')
			self.injectFieldNum.insert(0, str(self.remainingInjections))

	def reloadBalls(self):
		if self.reloadBallsNum.get().isdigit():
			self.remainingBalls += int(self.reloadBallsNum.get())
		if self.remainingBalls > 0:
			self.dropButton.config(state='normal')
		self.reloadBallsNum.delete(0, 'end')
		self.reloadBallsNum.insert(0, '1')

	def openHatch(self):
		self.opQueue.put(1)

	def closeHatch(self):
		self.opQueue.put(2)

	def driveOn(self):
		self.opQueue.put(4)

	def driveOff(self):
		self.opQueue.put(5)

	def rotate(self):
		self.opQueue.put(3)

	def inject(self):
		self.opQueue.put(6)

	def reloadInject(self):
		self.opQueue.put(7)

	def flush(self):
		self.opQueue.put(8)

	def calibrate(self):
		self.opQueue.put(100)

	def estop(self):
		operation_client(255)

	def updateInjectList(self):
		if self.remainingInjections == 0:
			self.injectFieldButton.config(state='disabled')
		else:
			self.injectFieldButton.config(state='normal')

	def handleHeartbeat(self, heartbeat):
	#Check for idle
		if heartbeat.currentOpCode == 0:
			self.idle = 1
		else:
			self.idle = 0

	#Heartbeat Frame
		self.currOp.set(self.opLookup(heartbeat.currentOpCode))
		self.lastOp.set(self.opLookup(heartbeat.opCodeOfLastAction))
		self.lastOpNum = heartbeat.opCodeOfLastAction
		if heartbeat.calibrated:
			self.calibratedVal.config(text="Yes", foreground="green")
		else:
			self.calibratedVal.config(text="No", foreground="red")
		if heartbeat.hatchOpen:
			self.hatchStateVal.config(text="Open", foreground="green")
		else:
			self.hatchStateVal.config(text="Closed", foreground="red")
		if heartbeat.wheelInPosition:
			self.wheelStateVal.config(text="In Position", foreground="green")
		else:
			self.wheelStateVal.config(text="Out of Position", foreground="red")
		if heartbeat.driverForward:
			self.driverStateVal.config(text="Forward")
		else:
			self.driverStateVal.config(text="Back")

	#Motor Frame
		if heartbeat.rotateMotorOn:
			self.currMotor.set("Rotate")
		elif heartbeat.driveMotorOn:
			self.currMotor.set("Drive")
		elif heartbeat.injectMotorOn:
			self.currMotor.set("Inject")
		else:
			self.currMotor.set("None")

		self.duration.set(str(heartbeat.actionDuration) + " ms")
		self.counterStart.set(heartbeat.counterStartVal)
		self.counterEnd.set(heartbeat.counterEndVal)
		self.instCurrent.set(str(heartbeat.instantaneousCurrent)+" mA")
		self.minCurrent.set(str(heartbeat.lowestCurrent)+" mA")
		self.maxCurrent.set(str(heartbeat.highestCurrent)+" mA")
		self.totCurrent.set(str(heartbeat.totalCurrent)+" mA")

	#Warning Frame
		if heartbeat.fireDanger:
			self.fireWarningVal.config(text="Fire Danger", foreground="red")
		elif heartbeat.criticalFireDanger:
			self.fireWarningVal.config(text="CRITICAL FIRE DANGER!", foreground="red")
		else:
			self.fireWarningVal.config(text="None", foreground="green")

		failures=0
		failStr = ""
		if heartbeat.hatchFailure:
			failStr += "hatch"
			failures += 1
		if heartbeat.wheelFailure:
			if failures > 0:
				failStr += ", "
			failStr += "wheel"
			failures += 1
		if heartbeat.driverFailure:
			if failures > 0:
				failStr += ", "
			failStr += "driver"
			failures += 1
		if heartbeat.injectorFailure:
			if failures > 0:
				failStr += ", "
			failStr += "injector"
			failures += 1

		if failures > 0:
			self.hardwareFailure.set(failStr)
		else:
			self.hardwareFailure.set("None")

#Variable upkeep
		self.remainingInjections = heartbeat.remainingInjections
		self.remainingInjectionsStr.set(str(self.remainingInjections))
		self.remainingBallsStr.set(str(self.remainingBalls))

#Connectivity monitoring
		currFreq = 1.0E9/(heartbeat.header.stamp.nsecs-self.lastHbTimestamp)
		self.freq = self.filterWeight*currFreq + (1-self.filterWeight)*self.freq
		self.lastHbTimestamp = heartbeat.header.stamp.nsecs
		self.timestampVal.config(text=str(heartbeat.header.stamp.secs))
		self.freqVal.config(text=str(int(self.freq))+' Hz')
		text = 'Ok'
		fg = 'green'
		if self.freq < 1:
			text = 'Lost connection!'
			fg = 'red'
		self.connVal.config(text=text, foreground=fg)

	def opLookup(self, opCode):
		if opCode == 1:
			return "Open Hatch"
		elif opCode == 2:
			return "Close Hatch"
		elif opCode == 3:
			return "Rotate"
		elif opCode == 4:
			return "Drive On"
		elif opCode == 5:
			return "Drive Off"
		elif opCode == 6:
			return "Inject"
		elif opCode == 7:
			return "Reload Inject"
		elif opCode == 8:
			return "Flush"
		elif opCode == 100:
			return "Calibrate"
		elif opCode == 255:
			return "ESTOP"

	def op_timer_callback(self, event):
		if not self.opQueue.empty() and self.idle:
			self.idle = 0
			nextOp = self.opQueue.get()
			print(self.lastOpNum)
			if nextOp == 3 and self.ballInPosition:
				self.remainingBalls -= 1
			operation_client(nextOp)

	def setDCounter(self):
		print("Counter Set: %s"%str(self.dCounter.get()))

	def setRCounter(self):
		print("Counter Set: %s"%str(self.rCounter.get()))

	def setICounter(self):
		print("Counter Set: %s"%str(self.iCounter.get()))

	def setDThresh(self):
		print("Low Threshold Set: %s"%str(self.dLowThresh.get()))
		print("High Threshold Set: %s"%str(self.dHighThresh.get()))

	def setRThresh(self):
		print("Threshold Set: %s"%str(self.rThresh.get()))

	def setIThresh(self):
		print("Threshold Set: %s"%str(self.iThresh.get()))

	

def main():
	hatchOpen = 0

	root = Tk()
	root.geometry("+300+300")
	app = BallDropperGUI(root)
	root.mainloop()

if __name__ == '__main__':
	main()