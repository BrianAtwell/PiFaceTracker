


import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import Tkinter as tk
import ttk as ttk

from pidcontroller import PIDController

"""
Proportional: 0.05
Integral: 0.15
Derivative: 0.042
"""
DefaultProportional= 0.05
DefaultIntegral=0.15
DefaultDerivative=0.042

pidControl=PIDController()

class PIDGrapher(tk.Tk):

	def __init__(self, *args, **kwargs):
		
		tk.Tk.__init__(self, *args, **kwargs)

		#tk.Tk.iconbitmap(self, default="clienticon.ico")
		tk.Tk.wm_title(self, "PID Grapher")
		
		
		self.mainFrame = tk.Frame(self)
		self.mainFrame.grid_rowconfigure(0, weight=1)
		self.mainFrame.grid_columnconfigure(0, weight=1)
		self.mainFrame.pack(side="top", fill="both", expand = True)
		
		self.toolbarFrame = ToolbarFrame(self.mainFrame, self)
		self.toolbarFrame.grid(row=0, column=0, sticky="nsew")
		
		self.plotFrame = PlotFrame(self.mainFrame, self)
		self.plotFrame.grid(row=1, column=0, sticky="nsew")
		
		self.toolbarFrame.applyButton()

	def updateGraph(self):
		self.plotFrame.updatePlot()

		
class PlotFrame(tk.Frame):

	def __init__(self, parent, controller):
		tk.Frame.__init__(self, parent)
		
		self.grid_rowconfigure(0, weight=1)
		self.grid_columnconfigure(0, weight=1)

		self.figure = Figure(figsize=(5,5), dpi=100)
		self.subPlot = self.figure.add_subplot(111)
		list2=[5,6,1,3,8,9,3,5]
		self.subPlot.plot([1,2,3,4,5,6,7,8],list2)
		

		self.canvas = FigureCanvasTkAgg(self.figure, self)
		self.canvas.draw()
		#canvas.get_tk_widget().grid(row=1, column=0, sticky="w")
		self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

		self.toolbar = NavigationToolbar2Tk(self.canvas, self)
		self.toolbar.update()
		self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
		
	def updatePlot(self):
		self.figure.clear()
		self.subPlot = self.figure.add_subplot(111)
		delayList=[]
		list2=[]
		midPoint = 375.0
		pidControl.reset(startPoint=midPoint)
		#pidControl.initializeIntegral(startPoint=midPoint)
		delay=1
		pidControl.setSetPoint(0.0)
		output=midPoint
		measuredInput=output-midPoint
		for I in range(0,99):
			output = pidControl.update(measuredInput)
			list2.append(output)
			measuredInput=output-midPoint
			"""
			if I < delay:
				if I == 0:
					delayList.append(pidControl.update(pidControl.output))
				else:
					delayList.append(pidControl.update(delayList[0]))
				list2.append(delayList[0])
			else:
				if I == 0:
					delayList.append(pidControl.update(pidControl.output))
				else:
					delayList.append(pidControl.update(delayList[0]))
				list2.append(delayList[0])
				del delayList[0]
				for J in range(0,len(delayList)):
					delayList[J]
			"""
			
		self.subPlot.plot(range(0,99),list2)
		self.canvas.draw()
		
		
class EdibleAtrribute(tk.Frame):
	def __init__(self, parent, text, defaultValue=0.0):
		tk.Frame.__init__(self, parent)
		
		self.text=text
		
		self.grid_rowconfigure(0, weight=1)
		self.grid_columnconfigure(0, weight=1)
		self.grid_columnconfigure(1, weight=1)
		
		self.label = tk.Label(self, text=self.text)
		self.label.grid(row=0, column=0, sticky="NWES")
		self.label.pack(pady=10,padx=10, side="left")
		
		self.value = tk.StringVar()
		
		self.spinBox = tk.Spinbox(self, from_=0, to=10, textvariable=self.value)
		self.spinBox.grid(row=0, column=1, sticky="NWES")
		self.value.set(defaultValue)
		self.spinBox.pack(pady=10,padx=10)
		
	def get(self):
		return self.spinBox.get()
	
		
class ToolbarFrame(tk.Frame):

	def __init__(self, parent, controller):
		tk.Frame.__init__(self, parent)
		
		self.controller = controller
		
		self.proportionBox = EdibleAtrribute(self, "Proportional:", defaultValue=DefaultProportional)
		self.proportionBox.grid(row=0, column=0, sticky="nsew")
		self.proportionBox.pack()
		
		self.integralBox = EdibleAtrribute(self, "Integral:", defaultValue=DefaultIntegral)
		self.integralBox.grid(row=1, column=0, sticky="nsew")
		self.integralBox.pack()
		
		self.derivativeBox = EdibleAtrribute(self, "Derivative:", defaultValue=DefaultDerivative)
		self.derivativeBox.grid(row=1, column=0, sticky="nsew")
		self.derivativeBox.pack()
		
		applyBtn = ttk.Button(self, text="Apply",
					command=lambda: self.applyButton())
		applyBtn.pack()
		
		self.grid_rowconfigure(0, weight=1)
		self.grid_rowconfigure(1, weight=1)
		self.grid_columnconfigure(0, weight=1)
		self.grid_columnconfigure(1, weight=1)
		
	def applyButton(self):
		pidControl.setProportional(float(self.proportionBox.get()))
		pidControl.setIntegral(float(self.integralBox.get()))
		pidControl.setDerivative(float(self.derivativeBox.get()))
		self.controller.updateGraph()

		

app = PIDGrapher()
app.mainloop()