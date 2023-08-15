import sys, os.path
scanner_dir = (os.path.abspath(os.path.join(os.path.dirname(__file__), 'scanner_pyfiles')))

sys.path.append(scanner_dir)
# import setup_dotnet

from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from time import gmtime, strftime


import numpy as np
import threading
# import cv2
# from PIL import ImageTk, Image
import time

from utils.Robot_PC_comm import abb_connector
# from visionLib.camera_utils import CapturingModule
# from visionLib import vision_utils as visu


root  =  Tk()  # create root window
root.title("Colony Identification and streaking")
root.geometry("950x800")
#root.maxsize(900,  600)  # width x height

#if the size of the GUI window is fixed, need to fix the size of the frames.

# class GUITemp:
#     def __init__(self, master):

#         # initialize all the robot and camera connection
#         self.robotControl = None
#         self.cap = None

#         #buffer variable for the colony reference

#         self.roi_mask = cv2.imread("Images\\ROI_mask.png", 0)
#         self.cell_id_mask = cv2.imread("Images\\cell_id_mask.png", 0)
#         self.coefflist = np.load("visionLib\\coeff.npy")
#         self.sample_reference = None
#         self.sample_reference_list = None
#         self.image_vis = None
#         self.detectedColony = None
#         self.output_coordinate_list = None

#         self.running = True
#         self.runGetRef = False
#         self.runWithRef = False

#         mainProcessThread = threading.Thread(target=self.processCVTask, args=())
#         mainProcessThread.daemon = True
#         mainProcessThread.start()
        
#         #Global variables maintatined on the GUI as well as the robot controller
#         self.global_tip_count = 0

#         # GUI start here
#         #Main frame - Level0
#         self.frame = Frame(master)
#         self.frame.pack()

#         # Main frame -> Button panel frame 
#         self.button_panel_frame = LabelFrame(self.frame, text = "Control Panel",padx=15, pady=15)
#         self.button_panel_frame.grid(row=0 ,column=0, sticky='w'+'e'+'n'+'s')


#         # Main frame -> Button panel frame -> Scanner control panel
#         self.scanner_control_panel = LabelFrame(self.button_panel_frame, text = "Scanner Control", padx=15, pady=15)
#         self.scanner_control_panel.grid(row=0,column=0, sticky='w'+'e'+'n'+'s')
#         self.scanner_control_panel_ON = Button(self.scanner_control_panel, text="Scanner ON", padx=10, pady=10, command=lambda: threading.Thread(target = self.scanner_read).start())
#         self.scanner_control_panel_ON.grid(row=0,column=0)

#         self.scanner_control_panel_OFF = Button(self.scanner_control_panel, text="Scanner OFF", padx=10, pady=10, command=lambda: self.scanner_read())
#         self.scanner_control_panel_OFF.grid(row=1, column=0)

#         self.scanner_selection_label = Label(self.scanner_control_panel, text="Barcode location")
#         self.scanner_selection_label.grid(row=2, column=0)

#         self.barcode_selection=ttk.Combobox(self.scanner_control_panel, values =["","Input","Output"] )
#         self.barcode_selection.grid(row=3, column=0)

#         # Main frame -> Button panel frame -> Robot control panel
#         self.robot_control_panel = LabelFrame(self.button_panel_frame, text="Robot Process Control",padx=15, pady=15)
#         self.robot_control_panel.grid(row=1,column=0, sticky='w'+'e'+'n'+'s')

#         self.robot_control_START = Button(self.robot_control_panel, text="Robot START", padx=10, pady=10, command=lambda: threading.Thread(
#             target=self.robot_control_START_process).start(), state="disabled")
#         self.robot_control_START.grid(row=0,column=0)

#         self.robot_control_STOP = Button(self.robot_control_panel, text="Robot STOP",
#                                          padx=10, pady=10, command=self.robot_control_STOP_button, state="disabled")
#         self.robot_control_STOP.grid(row=1, column=0)

#         # Main frame -> Button panel frame -> Selection Panel
#         self.selection_control_panel = LabelFrame(self.button_panel_frame, text="Process Selection",padx=15, pady=15)
#         self.selection_control_panel.grid(row=0,column=1, sticky='w'+'e'+'n'+'s')


#         self.output_type_label01 = Label(self.selection_control_panel, text="Output_type")
#         self.output_type_label01.grid(row=0,column=0)

#         self.output_type = ttk.Combobox(self.selection_control_panel, values =["","Deep Well","Agar Plate"])
#         self.output_type.grid(row=1,column=0)

#         self.output_type.bind("<<ComboboxSelected>>", self.output_type_selection) # bind helps to call functions where conditions are set to enable or disable selection based on the master selection


#         self.output_type_label02 = Label(self.selection_control_panel, text="Streaking Pattern")
#         self.output_type_label02.grid(row=2,column=0)


#         self.streaking_pattern_selection=ttk.Combobox(self.selection_control_panel, values =["Streaking Pattern 1","Streaking Pattern 2"], state="disabled" )
#         self.streaking_pattern_selection.grid(row=3, column=0)

#         self.output_type_label03 = Label(self.selection_control_panel, text="Transfer_type")
#         self.output_type_label03.grid(row=4, column=0)

#         self.transferType_selection=ttk.Combobox(self.selection_control_panel, values =["1 to 1","1 to Many"] )
#         self.transferType_selection.grid(row=5, column=0)


#         # Main frame -> Button panel frame -> Data transfer
#         self.data_transfer_panel = LabelFrame(self.button_panel_frame, text="Data Transfer",padx=15, pady=15)
#         self.data_transfer_panel.grid(row=1,column=1, sticky='w'+'e'+'n'+'s')

#         self.check_server = Button(self.data_transfer_panel, text="CONNECT", padx=10, pady=10, command=self.enable_server)
#         self.check_server.grid(row=0,column=0)

#         self.push_data_button= Button(self.data_transfer_panel, text="TRANSFER", padx=10, pady=10, command=self.tranfer_data)
#         self.push_data_button.grid(row=1, column=0)

#         # Main frame -> Button panel frame -> Global Variables
#         self.global_variable_panel = LabelFrame(self.button_panel_frame, text="Global Variables",padx=15, pady=15)
#         self.global_variable_panel.grid(row=2,column=0, sticky='w'+'e'+'n'+'s')
        
#         self.tip_count = IntVar()
#         self.tip_counter_name = Label(self.global_variable_panel, text="TIP COUNT", padx=10, pady=10)
#         self.tip_counter_name.grid(row=0,column=0)

#         self.tip_count_value = Label(self.global_variable_panel, textvariable=self.tip_count, padx=10, pady=10)
#         self.tip_count_value.grid(row=0, column=1)

#         self.lblReferenceID = Label(self.global_variable_panel, text="Reference ID", padx=10, pady=10)
#         self.lblReferenceID.grid(row=1, column=0)

#         self.txtReferenceID = Text(self.global_variable_panel, height=1, width=2, padx=10, pady=10)
#         self.txtReferenceID.grid(row=1, column=1)

#         self.txtReferenceID.bind('<Return>', self.txtReferenceID_func)

#         self.camera_function_panel = LabelFrame(self.button_panel_frame, text="Camera Function", padx=15, pady=15)
#         self.camera_function_panel.grid(row=2, column=1, sticky='w'+'e'+'n'+'s')

#         self.detectrefBtn = Button(
#             self.camera_function_panel, text="Select Ref", padx=10, pady=10, command=self.detectReferenceColony, state="disabled")
#         self.detectrefBtn.grid(row=0, column=0)

#         self.detectColBtn = Button(
#             self.camera_function_panel, text="Auto Detect", padx=10, pady=10, command=self.detectColonyFromRef, state="disabled")
#         self.detectColBtn.grid(row=0, column=1)

#         self.swingUpBtn = Button(self.camera_function_panel, text="swingup", padx=10, pady=10, command=lambda:self.robotControl.set_dio(value=True,IOname='Ring_motor'), state="normal")
#         self.swingUpBtn.grid(row=1, column=0)

#         self.swingDownBtn = Button(self.camera_function_panel, text="swingDown", padx=10, pady=10, command=lambda:self.robotControl.set_dio(value=False,IOname='Ring_motor'), state="normal")
#         self.swingDownBtn.grid(row=1, column=1)
#         ####################################

#         # Main frame -> Image panel frame 
#         Image_panel_frame = LabelFrame(self.frame, text = "Image Viewer",padx=15, pady=15)
#         Image_panel_frame.grid(row=0 ,column=1)

#         # Include Image display with 'Exception handler' so that a default image always displayed
#         Default_img = ImageTk.PhotoImage(Image.open("Images\\Blank.png")) # path to default image can be changed accordingly.
#         self.Image_label = Label(Image_panel_frame, image= Default_img)
#         self.Image_label.grid(row = 0,column=1)


#         #Main frame -> Data table
#         self.Data_table_frame = LabelFrame(self.frame, text = "Data Table",padx=15, pady=15, height= 300 )
#         self.Data_table_frame.grid(row=1, columnspan=2, sticky='WE' )
        
#         #setting up scroll bar for the data table
#         vScollBar = ttk.Scrollbar(self.Data_table_frame, orient ="vertical")
#         #vScollBar.grid(row =0, column=1, sticky='ns')

#         hScrollBar = ttk.Scrollbar(self.Data_table_frame, orient ="horizontal")
#         #hScrollBar.grid(row =1, columnspan=2, sticky='WE')
        
#         self.Input_table = ttk.Treeview(self.Data_table_frame, yscrollcommand=vScollBar.set, xscrollcommand=hScrollBar.set, height=7)
#         vScollBar.config(command=self.Input_table.yview)
#         hScrollBar.config(command=self.Input_table.xview)

#         vScollBar.place(x=760,y=0, height =180) # the scroll bar length needs to be varied when table length is varied
#         hScrollBar.place(x=0,y=180, width =750)

#         #the Data tracing table columns from first discussion
#         # columns are 'Run no.??', 'Incoming plate number', 'Plate position no','colony detected','Success of colony detected','Streaking pattern','Outgoing plate no','Plate position no' 
#         self.Input_table['columns'] = ('IN Barcode', 'OUT Barcode', 'Transfer type', 'Output transfer','IN pick positions(cell no.)','OUT streak positions(cell no.)','TimeStamp','Colony type','Colony colour','Colony size(Dia)','Colony texture')

#         self.Input_table.column("#0", width=0,  stretch=NO)
#         self.Input_table.heading("#0",text="",anchor=CENTER)

#         for column in self.Input_table['columns']:
#             self.Input_table.column(column,anchor=CENTER, width=100,stretch=NO )
#             self.Input_table.heading(column,text=column, anchor=CENTER)

#         # self.Input_table.column("IN Barcode",anchor=CENTER, width=100,stretch=NO) 
#         # self.Input_table.column("OUT Barcode",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("Transfer type",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("Output transfer",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("IN pick positions(cell no.)",anchor=CENTER,width=100,)
#         # self.Input_table.column("OUT streak positions(cell no.)",anchor=CENTER,width=100)
#         # self.Input_table.column("TimeStamp",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("Colony type",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("Colony colour",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("Colony size(Dia)",anchor=CENTER,width=100,stretch=NO)
#         # self.Input_table.column("Colony texture",anchor=CENTER,width=100,stretch=NO)

#         # self.Input_table.heading("#0",text="",anchor=CENTER)
#         # self.Input_table.heading("IN Barcode",text="IN Barcode",anchor=CENTER)
#         # self.Input_table.heading("OUT Barcode",text="OUT Barcode",anchor=CENTER)
#         # self.Input_table.heading("Transfer type",text="Transfer type",anchor=CENTER)
#         # self.Input_table.heading("Output transfer",text="Output transfer",anchor=CENTER)
#         # self.Input_table.heading("IN pick positions(cell no.)",text="IN pick positions(cell no.)",anchor=CENTER)
#         # self.Input_table.heading("OUT streak positions(cell no.)",text="OUT streak positions(cell no.)",anchor=CENTER)
#         # self.Input_table.heading("TimeStamp",text="TimeStamp",anchor=CENTER)
#         # self.Input_table.heading("Colony type",text="Colony type",anchor=CENTER)
#         # self.Input_table.heading("Colony colour",text="Colony colour",anchor=CENTER)
#         # self.Input_table.heading("Colony size(Dia)",text="Colony size(Dia)",anchor=CENTER)
#         # self.Input_table.heading("Colony texture",text="Colony texture",anchor=CENTER)
        
#         self.rowCountDataTable = IntVar() # this variable is used to keep count of rows in the data table
#         self.rowCountDataTable.set(0)

#         #Input_table.grid(row=0, column=0 )
#         self.Input_table.place(x=0,y=0, width=750)

#         #Data entry fields to populate datatable
#         IN_barcode = Label(self.Data_table_frame, text = 'IN Barcode')
#         IN_barcode.place(x=15, y= 210, width = 100)

#         OUT_barcode = Label(self.Data_table_frame, text = 'OUT Barcode')
#         OUT_barcode.place(x=115, y= 210, width = 100)

#         Colony_type= Label(self.Data_table_frame, text = 'Colony type')
#         Colony_type.place(x=215, y= 210, width = 100)
        
#         Colony_colour= Label(self.Data_table_frame, text = 'Colony colour')
#         Colony_colour.place(x=315, y= 210, width = 100)

#         Colony_texture= Label(self.Data_table_frame, text = 'Colony texture')
#         Colony_texture.place(x=415, y= 210, width = 100)


#         self.IN_barcode_entry= Entry(self.Data_table_frame)
#         self.IN_barcode_entry.place(x=15, y= 230, width = 100)

#         self.OUT_barcode_entry = Entry(self.Data_table_frame)
#         self.OUT_barcode_entry.place(x=115, y= 230, width = 100)

#         self.Colony_type = Entry(self.Data_table_frame)
#         self.Colony_type.place(x=215, y= 230, width = 100)

#         self.Colony_colour = Entry(self.Data_table_frame)
#         self.Colony_colour.place(x=315, y= 230, width = 100)

#         self.Colony_texture = Entry(self.Data_table_frame)
#         self.Colony_texture.place(x=415, y= 230, width = 100)

#         # buttons for update and edit of data table for user input

#         self.edit_selected = False  # variable to check if edit option selected

#         update_data_button = Button(self.Data_table_frame, text='Update', command = self.updateDataTable)
#         update_data_button.place(x=530, y= 230, width = 70)

#         edit_data_button = Button(self.Data_table_frame, text='Edit', command = self.editDataTable)
#         edit_data_button.place(x=620, y= 230, width = 70)

#     def initialize_connection(self):
#         # intialize the connection with a robot
#         try:
#             self.robotControl = abb_connector.Robot()
#             self.robot_control_START.config(state="normal")
#             self.robot_control_STOP.config(state="normal")
#         except:
#             print("cannot establish connection with a robot")

#         try:
#             # initialize camera module
#             self.cap = CapturingModule()
#             self.cap.setup()

#             self.detectrefBtn.config(state="normal")
#         except:
#             print("cannot establish connection with a camera")

#     def closing_connection(self):
#         self.running = False
#         self.cap.close()
#         self.robotControl.close()

#     def scanner_read(self):
        
#         try:

#             scan_value = '101010'
#             #scan_value = setup_dotnet.Barcode_Scanner_Data()
            
#             if not self.barcode_selection.get() == "":


#                 if self.barcode_selection.get() == "Input":
#                     if not len(self.IN_barcode_entry.get()) == 0:
#                         self.IN_barcode_entry.delete(0,END)
#                         self.IN_barcode_entry.insert(INSERT, scan_value)
                    
#                     else:
#                         self.IN_barcode_entry.insert(INSERT, scan_value)
                    

#                 else:
#                     if not len(self.OUT_barcode_entry.get()) == 0:
#                         self.OUT_barcode_entry.delete(0,END)
#                         self.OUT_barcode_entry.insert(INSERT,scan_value)

#                     else:
#                         self.OUT_barcode_entry.insert(INSERT,scan_value)
            
#             else:
#                 messagebox.showerror('Barcode Location Error','Please selection the location of the barcode before scanning')

#         except:
#             #fill in the exception caused by barcode connection error
#             pass

#     def robot_control_START_process(self):
        
#         #check if input and output workplate barcodes are registered and prompt operator to register them if not 
#         #if start of the process

#         if self.output_type.get() == "":
#             messagebox.showerror('Selection Error', 'Please select the output type before starting process')

#         else:

#             #this is a dummy value set for the tip counter, but later needs to be input from a input field on GUI
#             abb_connector.global_tip_counter(self.robotControl, 1, 0)

#             self.global_tip_count = abb_connector.global_tip_counter(self.robotControl, 0) # get the tip_count from the Robot controller

#             self.tip_count.set(96 - self.global_tip_count) # set the GUI indicator for tip count
            
#             # get value based on selection
#             if self.output_type.get() == "Agar Plate":
#                 output_selected = self.streaking_pattern_selection.get()
#                 type_selected = self.transferType_selection.get()
                

#             else:
#                  output_selected = self.output_type.get()
#                  type_selected = self.transferType_selection.get()

#             print(output_selected)

#             self.robotControl.set_dio(True, 'Ring_motor')
#             self.robotControl.set_dio(False, 'Ring_light')
#             time.sleep(0.5)
            
#             new_value = abb_connector.Robot_start(self.robotControl, 
#                 self.global_tip_count, self.output_coordinate_list, output_selected, type_selected)
#             used_value = self.tip_count.get() - new_value
#             self.tip_count.set(used_value) # set the new value of the Tip counter after sequence

#             #set the Global_tip counter after process
#             abb_connector.global_tip_counter(self.robotControl, 1, self.global_tip_count + new_value)

#             #This portion sets the value in the datatable for the process completed
#             #Call function that updates values into the data table. See how the vision developed parmeters can be updated.
#             self.populteDataTable()

#     def robot_control_STOP_button(self):
        
#         return

#     def output_type_selection(self,e):
#         """
#         This function is used as a binding for the selection panel
#         """
#         if self.output_type.get() == "Agar Plate":
#             self.streaking_pattern_selection=ttk.Combobox(self.selection_control_panel, values =["Streaking Pattern 1","Streaking Pattern 2"], state="normal" )
#             self.streaking_pattern_selection.grid(row=3, column=0)
        
#         else:
#             self.streaking_pattern_selection=ttk.Combobox(self.selection_control_panel, values =["Streaking Pattern 1","Streaking Pattern 2"], state="disabled" )
#             self.streaking_pattern_selection.grid(row=3, column=0)

#     def txtReferenceID_func(self, e):
#         print(e)
#         if e.char == '\r':
#             input = self.txtReferenceID.get("1.0", END)
#             self.txtReferenceID.delete('1.0', END)
#             try:
#                 ref_id = int(input)

#                 if not self.sample_reference_list is None:
#                     self.sample_reference = self.sample_reference_list[ref_id]

#                     i, _ = self.sample_reference
#                     image_vis = self.image_vis.copy()
#                     center = (i[0], i[1])
#                     # circle center
#                     cv2.circle(image_vis, center, 1, (0, 0, 255), 10)
#                     # circle outline
#                     radius = i[2]
#                     cv2.circle(image_vis, center, radius, (0, 0, 255), 10)

#                     image_vis = cv2.resize(image_vis, (0, 0), fx=0.1, fy=0.1)
#                     image_vis = cv2.cvtColor(image_vis, cv2.COLOR_BGR2RGB)

#                     image_from_camera = ImageTk.PhotoImage(Image.fromarray(image_vis))
#                     self.Image_label.configure(image=image_from_camera)
#                     self.Image_label.image = image_from_camera

#                     self.detectColBtn.config(state="normal")
#             except:
#                 return
#         return

#     def get3d_from_2dpoint(self, point):
#         x_3d = (point[0] * self.coefflist[0,0]) + (point[1] * self.coefflist[0,1]) + self.coefflist[0,2]
#         y_3d = (point[0] * self.coefflist[1,0]) + (point[1] * self.coefflist[1,1]) + self.coefflist[1,2]
#         z_3d = (point[0] * self.coefflist[2,0]) + (point[1] * self.coefflist[2,1]) + self.coefflist[2,2]
#         return [x_3d, y_3d, z_3d]

#     def calculate_cell_diameter(self, center, radius):
#         diagonal_two_points = ((center[0] - radius, center[1]), (center[0] + radius, center[1]))
#         horizontal_two_points = ((center[0], center[1] - radius), (center[0], center[1] + radius))
        
#         diagonal_diameter = ((np.array(self.get3d_from_2dpoint(diagonal_two_points[0])) - np.array(self.get3d_from_2dpoint(diagonal_two_points[1]))) ** 2).sum()**.5
#         horizontal_diameter = ((np.array(self.get3d_from_2dpoint(horizontal_two_points[0])) - np.array(self.get3d_from_2dpoint(horizontal_two_points[1]))) ** 2).sum()**.5

#         return (diagonal_diameter + horizontal_diameter)/2
    
#     def get_cellID(self, point):
#         return self.cell_id_mask[point[1], point[0]] - 1

#     def processCVTask(self):
#         while self.running:

#             if self.runGetRef or self.runWithRef:
#                 image = None
#                 start_time = time.time()
#                 while image is None or (time.time() - start_time < 2):
#                     image = self.cap.run()
#                 if image is None:
#                     self.runGetRef = False
#                     self.runWithRef = False
#                     print("cannot get image")
#                     continue

#                 if self.runGetRef:
#                     print("without reference")
#                     start_time = time.time()
                    
#                     image_vis = cv2.resize(image.copy(), (0, 0), fx=0.1, fy=0.1)
#                     image_vis = cv2.cvtColor(image_vis, cv2.COLOR_BGR2RGB)

#                     image_from_camera = ImageTk.PhotoImage(Image.fromarray(image_vis))
#                     self.Image_label.configure(image=image_from_camera)
#                     self.Image_label.image = image_from_camera

#                     print("something on GUI time: {:.4f} seconds".format(time.time() - start_time))

#                     start_time = time.time()

#                     masked_image = image.copy()
#                     masked_image[self.roi_mask == 0] = 0
#                     detected_circles = visu.circleColonyDetect(masked_image)
                    
#                     print("detection time: {:.4f} seconds".format(time.time() - start_time))

#                     final_circles = visu.loop_iou_filtering(detected_circles, final_circles=[])
#                     print("final_circles:", len(final_circles))

#                     print("filtering time: {:.4f} seconds".format(time.time() - start_time))

#                     circle_with_hist = visu.compute_colorHist(masked_image, final_circles)

#                     print("detection time: {:.4f} seconds".format(time.time() - start_time))

#                     self.sample_reference_list = circle_with_hist

#                     image_vis = image.copy()
#                     self.image_vis = image.copy()
#                     for idx, data in enumerate(circle_with_hist):
#                         i, _ = data
#                         center = (i[0], i[1])
#                         # circle center
#                         # cv2.circle(image_vis, center, 1, (0, 100, 100), 10)
#                         TEXT_FACE = cv2.FONT_HERSHEY_DUPLEX
#                         TEXT_SCALE = 3.0
#                         TEXT_THICKNESS = 6
#                         TEXT = str(idx)

#                         text_size, _ = cv2.getTextSize(TEXT, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)
#                         text_origin = (int(center[0] - text_size[0] / 2), int(center[1] + text_size[1] / 2))

#                         cv2.putText(image_vis, TEXT, text_origin, TEXT_FACE, TEXT_SCALE, (255, 0, 255), TEXT_THICKNESS, cv2.LINE_AA)

#                         # circle outline
#                         radius = i[2]
#                         cv2.circle(image_vis, center, radius, (255, 0, 255), 10)

#                     image_vis = cv2.resize(image_vis, (0, 0), fx=0.1, fy=0.1)
#                     image_vis = cv2.cvtColor(image_vis, cv2.COLOR_BGR2RGB)

#                     image_from_camera = ImageTk.PhotoImage(Image.fromarray(image_vis))
#                     self.Image_label.configure(image=image_from_camera)
#                     self.Image_label.image = image_from_camera

#                     self.runGetRef = False
                    
#                 elif self.runWithRef:
#                     print("with reference")

#                     image_vis = image.copy()
#                     image_vis = cv2.resize(image_vis, (0, 0), fx=0.1, fy=0.1)
#                     image_vis = cv2.cvtColor(image_vis, cv2.COLOR_BGR2RGB)

#                     image_from_camera = ImageTk.PhotoImage(Image.fromarray(image_vis))
#                     self.Image_label.configure(image=image_from_camera)
#                     self.Image_label.image = image_from_camera

#                     masked_image = image.copy()
#                     masked_image[self.roi_mask == 0] = 0
#                     detected_circles = visu.circleColonyDetect(masked_image)
#                     final_circles = visu.loop_iou_filtering(detected_circles, final_circles=[])
#                     circle_with_hist = visu.compute_colorHist(masked_image, final_circles)

#                     data_with_div = visu.loop_color_similarity( image, circle_with_hist, reference=self.sample_reference)

#                     def get_divergence(elem):
#                         return elem[2]
#                     data_with_div = sorted(data_with_div, key=get_divergence)
#                     image_vis = image.copy()
                    
#                     self.detectedColony = {}

#                     type_selected = self.transferType_selection.get()

#                     for data in data_with_div:
#                         i, hist, j_div, r_diff = data
#                         if j_div < 15:
#                             center = (i[0], i[1])
#                             radius = i[2]
                            
#                             cell_id = self.get_cellID(center)
#                             if not cell_id in self.detectedColony.keys():
#                                 diameter3d = self.calculate_cell_diameter(center, radius)
#                                 center3d = self.get3d_from_2dpoint(center)
#                                 self.detectedColony[cell_id] = (center, radius, center3d, diameter3d)

#                                 # circle center
#                                 cv2.circle(image_vis, center, 1, (0, 100, 100), 3)
#                                 # circle outline
#                                 cv2.circle(image_vis, center, radius, (0, 255, 0), 3)

#                                 if type_selected == "1 to Many":
#                                     break
#                         else:
#                             break
#                             # break

#                     image_vis = cv2.resize(image_vis, (0, 0), fx=0.1, fy=0.1)
#                     image_vis = cv2.cvtColor(image_vis, cv2.COLOR_BGR2RGB)
#                     image_from_camera = ImageTk.PhotoImage(Image.fromarray(image_vis))
#                     self.Image_label.configure(image=image_from_camera)
#                     self.Image_label.image = image_from_camera

#                     output_coordinate_list = []
#                     for i in range(8):
#                         if not i in self.detectedColony.keys():
#                             output_coordinate_list.append((i, "Null"))
#                         else:
#                             output_coordinate_list.append((i, [self.detectedColony[i][2], [0.107, 0.087, 0.990, -0.009]]))

#                     self.output_coordinate_list = output_coordinate_list
#                     print(self.output_coordinate_list)

#                     #self.populteDataTable()

#                     self.runWithRef = False

#     def detectReferenceColony(self):
#         #Before capture image, make sure Ring light assembly turned on.
#         self.robotControl.set_dio(False, 'Ring_motor')
#         time.sleep(0.5)
#         self.robotControl.set_dio(True, 'Ring_light')
#         time.sleep(3)

#         self.runGetRef = True
    
#     def detectColonyFromRef(self):
#         self.runWithRef = True

#     def enable_server(self):
#         return

#     def tranfer_data(self):
#         return

#     def tipCounterReset(self):
#         abb_connector.global_tip_counter(self.robotControl, 1,0) # the value '0' indicates that the tip holder is full.
#         return

#     def enable_buttons(self):
#         self.robot_control_START = Button(self.robot_control_panel, text="Robot START", padx=10, pady=10, command=lambda: threading.Thread(target= self.robot_control_START_process).start(), state = "normal")
#         self.robot_control_START.grid(row=0,column=0)

#         #self.robot_control_STOP= Button(self.robot_control_panel, text="Robot STOP", padx=10, pady=10, command=self.robot_control_STOP_button, state = "normal")
#         #self.robot_control_STOP.grid(row=1, column=0)

#         self.scanner_control_panel_ON = Button(self.scanner_control_panel, text="Scanner ON", padx=10, pady=10, command=lambda: threading.Thread(target = self.scanner_read).start(), state = "normal")
#         self.scanner_control_panel_ON.grid(row=0,column=0)

#         self.scanner_control_panel_OFF = Button(self.scanner_control_panel, text="Scanner OFF", padx=10, pady=10, command=lambda: self.scanner_read(), state = "normal")
#         self.scanner_control_panel_OFF.grid(row=1, column=0)

#         self.check_server = Button(self.data_transfer_panel, text="CONNECT", padx=10, pady=10, command=self.enable_server, state = "normal")
#         self.check_server.grid(row=0,column=0)

#         self.push_data_button= Button(self.data_transfer_panel, text="TRANSFER", padx=10, pady=10, command=self.tranfer_data, state = "normal")
#         self.push_data_button.grid(row=1, column=0)

#         self.check_server = Button(self.camera_function_panel, text="Select Ref", padx=10, pady=10, command=self.detectReferenceColony, state = "normal")
#         self.check_server.grid(row=0, column=0)

#         self.check_server = Button(
#             self.camera_function_panel, text="Auto Detect", padx=10, pady=10, command=self.detectColonyFromRef, state = "normal")
#         self.check_server.grid(row=0, column=1)

#     def disable_buttons(self):
#         self.robot_control_START = Button(self.robot_control_panel, text="Robot START", padx=10, pady=10, command=lambda: threading.Thread(target= self.robot_control_START_process).start(), state = "disabled")
#         self.robot_control_START.grid(row=0,column=0)

#         #self.robot_control_STOP= Button(self.robot_control_panel, text="Robot STOP", padx=10, pady=10, command=self.robot_control_STOP_button, state = "normal")
#         #self.robot_control_STOP.grid(row=1, column=0)

#         self.scanner_control_panel_ON = Button(self.scanner_control_panel, text="Scanner ON", padx=10, pady=10, command=lambda: threading.Thread(target = self.scanner_read).start(), state = "disabled")
#         self.scanner_control_panel_ON.grid(row=0,column=0)

#         self.scanner_control_panel_OFF = Button(self.scanner_control_panel, text="Scanner OFF", padx=10, pady=10, command=lambda: self.scanner_read(), state = "disabled")
#         self.scanner_control_panel_OFF.grid(row=1, column=0)

#         self.check_server = Button(self.data_transfer_panel, text="CONNECT", padx=10, pady=10, command=self.enable_server, state = "disabled")
#         self.check_server.grid(row=0,column=0)

#         self.push_data_button= Button(self.data_transfer_panel, text="TRANSFER", padx=10, pady=10, command=self.tranfer_data, state = "disabled")
#         self.push_data_button.grid(row=1, column=0)

#         self.check_server = Button(self.camera_function_panel, text="Select Ref", padx=10, pady=10, command=self.detectReferenceColony, state = "disabled")
#         self.check_server.grid(row=0, column=0)

#         self.check_server = Button(
#             self.camera_function_panel, text="Auto Detect", padx=10, pady=10, command=self.detectColonyFromRef, state = "disabled")
#         self.check_server.grid(row=0, column=1)

#     def updateDataTable(self):
#         '''
#         This function is used to update the datatable for the User Inputs.
        
#         '''
#         try:

#             if self.edit_selected:

#                 selected=self.Input_table.focus()
                
#                 #grab exisitng data 
#                 table_values = list(self.Input_table.item(selected,'values'))
#                 table_values[0] = self.IN_barcode_entry.get()
#                 table_values[1] = self.OUT_barcode_entry.get()
#                 table_values[7] = self.Colony_type.get()
#                 table_values[8] = self.Colony_colour.get()
#                 table_values[10] = self.Colony_texture.get()

#                 # replace exisitng data with updated data from entry field
#                 self.Input_table.item(selected,text="",values=(table_values))
                
#                 #clear entry boxes
#                 self.IN_barcode_entry.delete(0,END)
#                 self.OUT_barcode_entry.delete(0,END)
#                 self.Colony_type.delete(0,END)
#                 self.Colony_colour.delete(0,END)
#                 self.Colony_texture.delete(0,END)
                
#                 self.edit_selected = False # reset the variable once edit operation is complete

#             else:
#                 rowCount = self.rowCountDataTable.get()

#                 self.Input_table.insert(parent='',index='end',iid = rowCount,text='',values=(self.IN_barcode_entry.get(),self.OUT_barcode_entry.get(),"","","","","",self.Colony_type.get(),self.Colony_colour.get(),"",self.Colony_texture.get()))
#                 #self.Input_table.item(str(rowCount),text="", values=(table_values))
            
#         except TclError:
#             messagebox.showerror('Entry Value Error', 'Value already in cell.Pleasae select "Edit" to change Values')

#     def editDataTable(self):
#         '''
#         This function enables the user to modify the user inputs based on the selected row
#         '''
#         self.edit_selected = True # set the variable for edit operation
        
#          #clear entry boxes
#         self.IN_barcode_entry.delete(0,END)
#         self.OUT_barcode_entry.delete(0,END)
#         self.Colony_type.delete(0,END)
#         self.Colony_colour.delete(0,END)
#         self.Colony_texture.delete(0,END)
        
#         #grab record
#         selected=self.Input_table.focus()
#         #grab record values
#         table_values = self.Input_table.item(selected,'values')
#         #temp_label.config(text=selected)

#         #output to entry boxes
#         self.IN_barcode_entry.insert(0,table_values[0])
#         self.OUT_barcode_entry.insert(0,table_values[1])
#         self.Colony_type.insert(0,table_values[7])
#         self.Colony_colour.insert(0,table_values[8])
#         self.Colony_texture.insert(0,table_values[10])

#     def populteDataTable(self):
#         '''
#         This function updates the datatable for each process cycle. 
        
#         '''
        
#         rowCount = self.rowCountDataTable.get()
#         table_values = list(self.Input_table.item(str(rowCount),'values'))

#         cell_id_input = None
#         cell_id_output = None
#         cell_diameter3d = None
#         for i in range(8):
#             if i in self.detectedColony.keys():
#                 if cell_id_input is None:
#                     cell_id_input = str(i)
#                     cell_id_output = str(i)
#                     cell_diameter3d = self.detectedColony[i][3]
#                 else:
#                     cell_id_input += ","+str(i)
#                     cell_id_output += ","+str(i)
#         # from vision algorithm:
#         #1. Input colony positions
#         table_values[4] = cell_id_input
#         #2. Output colony positions
#         table_values[5] = cell_id_output
#         #3. Colony Diameter
#         table_values[9] = cell_diameter3d

        
#         # from output selection
#         #1. Type_selection
#         table_values[2] = self.transferType_selection.get()

#         #2. Output_Type + streaking pattern
#         if self.output_type.get() == 'Agar Plate':
#             table_values[3] = self.output_type.get() + "-" + self.streaking_pattern_selection.get()
        
#         else:
#             table_values[3] = self.output_type.get()

#         # from logic or program sequence
#         #1. TimeStamp
#         table_values[6] = strftime("%Y-%m-%d %H:%M:%S", gmtime())

#         self.Input_table.item(str(rowCount),text="",values=(table_values))

#         # row incrementer in table
#         rowCount += 1
#         self.rowCountDataTable.set(rowCount)        


####################################################################################################################

# For SILA : Standardization in Lab Automation

####################################################################################################################

class SiLA:
    def __init__(self):
        self.Robot_connect = abb_connector.Robot()

    def ringLightMotor(self, position):
        '''
        Input arguements:
        1. position : (Boolean)
            True - Ring light motor moves assembly to upright position
            False - Ring light motor moves assembly above the colony sample

        Return: None
        '''
        self.Robot_connect.set_dio(position, 'Ring_motor')
        
        return None

    def ringLightLamp(self,status):
        '''
        Input arguements:
        1. status : (Boolean)
            True - Ring light lamp turns ON
            False - Ring light lamp turns OFF

        Return: None
        
        '''
        self.Robot_connect.set_dio(status, 'Ring_light')
        #print(value)

        return None

    def getTipCount(self):
        '''
        Input arguements: None
        
        Return: Integer - remaining tips in the tip station

        '''
        
        tip_count = abb_connector.global_tip_counter(self.robotControl, 0)
        tip_count = 96 - tip_count

        return tip_count

    def setTipCount(self,value = 96):

        '''
        Input arguements: 
        1. value: Interger value for resetting the tip station. Value can be less than the 96 which is available 
            e.g.: If all tip slots are filled, entry value = 96.
            if no value is input, by deafualt this function resets the tip station to to represent full reset.
        
        Return: None

        '''
        if value <= 96:
            value = 96 - value
            abb_connector.global_tip_counter(self.robotControl, 1, 0)

            return None 

        else:
            
            return 0


if __name__ == '__main__':

    #initialie GUI class
    #GUI = GUITemp(root)

    # root.after(100, GUI.initialize_connection)
    # # root.protocol("WM_DELETE_WINDOW", GUI.closing_connection)
    # root.mainloop()

    # GUI.closing_connection()

    new = SiLA()
    new.ringLightMotor(False)
    new.ringLightLamp(True)
    

