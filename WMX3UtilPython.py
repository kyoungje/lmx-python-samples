# Import WMX3 API library
from WMX3ApiPython import *
from WMX3ApiPython import constants

# Import Python libraries and declare utility functions
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

# import multiprocessing
import threading
from threading import Event
from time import sleep
import psutil
import os

# from multiprocessing.managers import BaseManager
# class WMX3PyManager(BaseManager):
#     pass

# WMX3PyManager.register('Log', Log)
# wmx3_manager = WMX3PyManager()
# wmx3_manager.start()

# Global functions
def check_errorcode(func, error_code):
    if error_code != ErrorCode.PyNone:
        if error_code >= 0x00000 and error_code <= 0x10000:
            last_error_str =  f"function: {func}, ErrorType: WMX3Api, ErrorCode: {error_code}"
        elif error_code >= 0x11000 and error_code <= 0x11FFF:
            last_error_str = f"function: {func}, ErrorType: Log, ErrorCode: {error_code}"
        else:
            last_error_str = f"function: {func}, ErrorType: Undefined, ErrorCode: {error_code}"
        
        print(last_error_str)
        raise RuntimeError(last_error_str)


# Constants
INFINITE = int(0xFFFFFFFF)
HISTORY_INDEX_POS = 0
HISTORY_INDEX_VEL = 1

class WMX3PyUtil:
    def __init__(self, wmx3Lib):
        self.wmxLog = Log(wmx3Lib)
        self.log_data_history = [np.zeros((0,)), np.zeros((0,))]
        self.current_log_channel = -1
        self.overflow_flag = 0

        # Initialize the stop_event for thread control
        self.stop_event = Event()
        
        self.initialize_plot_style()

    def initialize_plot_style(self):
        """Set up the plot style for visualizations."""        
        sns.set(style='whitegrid', palette='muted', font_scale=1.2)
        HAPPY_COLORS_PALETTE = ["#01BEFE", "#FFDD00", "#FF7D00", "#FF006D", "#ADFF02", "#8F00FF"]
        sns.set_palette(sns.color_palette(HAPPY_COLORS_PALETTE))
        self.title_font = {
            'fontsize': 16,
            'fontweight': 'bold'
        }

    def collect_logdata(self, channel, axis=0):
        updated_logdata = None

        ret, memLogData = self.wmxLog.GetMemoryLogData(channel)
        if ret != ErrorCode.PyNone:
            check_errorcode("GetMemoryLogData", ret)
        else:
            # TODO: Maybe due to the GIL, always overlfowFlag occurs at early reads
            # Need to change subprocess to read the logdata
            if memLogData.overflowFlag > 0:
                print(f'(WARNING) Log overflow detected!')                                            
                self.overflow_flag += 1
            
            updated_logdata = [None]*memLogData.count
            print(f'Read log buffer: {memLogData.count}')                            

            if memLogData.count > 0:
                for index in range(memLogData.count):
                    logdata = memLogData.GetLogData(index)                                      
                    updated_logdata[index] = logdata.GetLogAxisData(axis)
        
        return updated_logdata


    def update_log_task(self, channel, dump_flag=True):       
        try:
            while not self.stop_event.is_set():
                updated_logdata = self.collect_logdata(channel, 0)

                if not updated_logdata or len(updated_logdata) == 0:
                    # if no logdata, wait for a while     
                    sleep(0.001)
                else:
                    pos_value_array = [data.feedbackPos for data in updated_logdata]
                    vel_value_array = [data.feedbackVelocity for data in updated_logdata]

                    ret, memLogStatus = self.wmxLog.GetMemoryLogStatus(channel)    
                    if ret == ErrorCode.PyNone:
                        print(f"memLogStatus: {memLogStatus.logState}, {memLogStatus.samplesToCollect}, {memLogStatus.samplesCollected}, {memLogStatus.usageRate}")

                    self.log_data_history[HISTORY_INDEX_POS] = np.concatenate((self.log_data_history[HISTORY_INDEX_POS], pos_value_array), axis=0)
                    self.log_data_history[HISTORY_INDEX_VEL] = np.concatenate((self.log_data_history[HISTORY_INDEX_VEL], vel_value_array), axis=0)

                if dump_flag:
                    print(f'Read log buffer size: {len(updated_logdata)}')                                  

        except Exception as e:
             print(f"Error in update_log_task: {e}")

        if dump_flag:
            print(f'Updated data size: {self.log_data_history[HISTORY_INDEX_POS].size}, {self.log_data_history[HISTORY_INDEX_VEL].size}')
            print(self.log_data_history)
           
        print(f'MemoryLog thread stopped.')                                  


    def is_available_logchannel(self, channel):    
        ret, memLogStatus = self.wmxLog.GetMemoryLogStatus(channel)    
        if ret != ErrorCode.PyNone:
            check_errorcode("GetMemoryLogStatus", ret)
            return False
        
        # REVISIT: The bufferOpened flag is alway TRUE even after closing the buffer.
        if memLogStatus.logState != LogState.Idle and memLogStatus.bufferOpened == True:
            print(f"memLogStatus.LogState: {memLogStatus.logState}, memLogStatus.bufferOpened: {memLogStatus.bufferOpened}")
            return False
        else:
            return True

    def _check_memory_logchannel(self):
        channel = -1
        for current_channel in range(constants.maxLogChannel - 1, 1, -1):
            print(f"curchannel: {current_channel}")
            if self.is_available_logchannel(current_channel):
                ret = self.wmxLog.OpenMemoryLogBuffer(current_channel);
                if ret == ErrorCode.PyNone:
                    channel = current_channel   
                    break
                else:
                    check_errorcode("OpenMemoryLogBuffer", ret)                

        return channel

    def start_log(self):
        if self.current_log_channel != -1:
            self.stop_log(self.current_log_channel)
        
        MAX_AXES = 2
        channel = self._check_memory_logchannel()
        updaterProc = None
        
        if channel != -1:
            axisSel = AxisSelection()
            
            axisSel.axisCount = MAX_AXES;
            for idx in range(MAX_AXES):
                axisSel.SetAxis(idx, idx)

            # evi = CoreMotionEventInput()
            # evi.inputFunction  = CoreMotionEventInputType.OpState;
            # inputFuncArg = CoreMotionEventInputFunctionArguments_OpState()
            # inputFuncArg.opState = OperationState.Pos
            # inputFuncArg.axis = 0
            # inputFuncArg.invert = 0
            # evi.opState = inputFuncArg
            
            # # Set the output function to None
            # evo = EventApiEventOutput()
            # evo.outputFunction = EventApiEventOutputType.PyNone;
        
            # Set the event to trigger the memory log
            # ret, eventId = wmxEventCtrl.SetEvent(evi, evo)
            # if ret != ErrorCode.PyNone:
            #     check_errorcode("SetEvent", ret)            
            # else:
            #     memOption.triggerEventCount = 1
            #     memOption.SetTriggerEventID(0, eventId)
            #     wmxEventCtrl.EnableEvent(eventId, 1)
        
            memOption = MemoryLogOptions()
            memOption.triggerEventCount = 0

            ret = self.wmxLog.SetMemoryLog(channel, axisSel, memOption)
            if ret != ErrorCode.PyNone:
                check_errorcode("SetMemoryLog", ret)
                return -1
            
            ret = self.wmxLog.StartMemoryLog(channel)
            if ret != ErrorCode.PyNone:
                check_errorcode("StartMemoryLog", ret)
                return -1

            self.current_log_channel = channel

            # Clear the stop_event before starting a worker thread
            self.stop_event.clear()
            self.overflow_flag = 0
            self.log_data_history = [np.zeros((0,)), np.zeros((0,))]

            self.log_update_thread = threading.Thread(
                target=self.update_log_task, args=(channel, False)
            )
            self.log_update_thread.start()

               # Increase the priority of the worker thread
            current_process = psutil.Process(os.getpid())
            for thread in current_process.threads():
                if thread.id == self.log_update_thread.ident:
                    psutil.Process(thread.id).nice(psutil.HIGH_PRIORITY_CLASS)

            # Wait for a while to scheulde the worker thread
            sleep(0.001)

        return channel

    def stop_log(self, channel):
        ret, memLogStatus = self.wmxLog.GetMemoryLogStatus(channel)
        if ret != ErrorCode.PyNone:
            check_errorcode("GetMemoryLogStatus", ret)
            return False

        if memLogStatus.logState == LogState.Running:           
            ret = self.wmxLog.StopMemoryLog(channel)
            if ret != ErrorCode.PyNone:
                check_errorcode("StopMemoryLog", ret)
                return False
                       
        ret = self.wmxLog.CloseMemoryLogBuffer(channel);
        if ret != ErrorCode.PyNone:
            check_errorcode("CloseMemoryLogBuffer", ret)

        self.current_log_channel = -1
        return True

    def pause_log(self, channel):

        # Stop the update process
        self.stop_event.set()

        ret = self.wmxLog.StopMemoryLog(channel)
        if ret != ErrorCode.PyNone:
            check_errorcode("StopMemoryLog during PauseLog", ret)
        
        sleep(0.001)   
        
        if self.log_update_thread and self.log_update_thread.is_alive():
            self.log_update_thread.join()
    
        # Print the summary of updated logdata
        print(f'Collected log data size: {self.log_data_history[HISTORY_INDEX_POS].size}')
        print(f'Log Overflow: {self.overflow_flag}');


    def draw_plots(self, plot_title, dump_flag=False):         
        if dump_flag:
            print(f'Updated data size: {self.log_data_history[HISTORY_INDEX_POS].size}, {self.log_data_history[HISTORY_INDEX_VEL].size}')
            print(self.log_data_history)
            
        # Plot position and velocity feedbacks
        fig, axs = plt.subplots(
            nrows=1,
            ncols=2,
            sharey=False,
            sharex=False,
            figsize=(25, 15)
        )
        
        ax = axs[HISTORY_INDEX_POS]
        ax.plot(self.log_data_history[HISTORY_INDEX_POS], label='Position', color='limegreen')
        ax.set_xlabel('Cycle')
        ax.set_title(f'{plot_title}: Feedback Position', fontdict=self.title_font, pad=20)
        ax.ticklabel_format(useOffset=False)
        ax.legend(['Position'])
        
        ax = axs[HISTORY_INDEX_VEL]
        ax.plot(self.log_data_history[HISTORY_INDEX_VEL], label='Velocity', color='violet')
        ax.set_xlabel('Cycle')
        ax.set_title(f'{plot_title}: Feedback Velocity', fontdict=self.title_font, pad=20)
        ax.legend(['Velocity'])
        
        fig.tight_layout();