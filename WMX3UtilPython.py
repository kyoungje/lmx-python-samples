# Import WMX3 API library
from WMX3ApiPython import *
from WMX3ApiPython import constants

# Import Python libraries and declare utility functions
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

import multiprocessing
from multiprocessing import Process, Event
from time import sleep

# Global functions
def check_errorcode(func, error_code, error_queue=None):
    if error_code != ErrorCode.PyNone:
        if error_code >= 0x00000 and error_code <= 0x10000:
            last_error_str =  f"function: {func}, ErrorType: WMX3Api, ErrorCode: {error_code}"
        elif error_code >= 0x11000 and error_code <= 0x11FFF:
            last_error_str = f"function: {func}, ErrorType: Log, ErrorCode: {error_code}"
        else:
            last_error_str = f"function: {func}, ErrorType: Undefined, ErrorCode: {error_code}"
        
        if error_queue:
            error_queue.put(last_error_str)
        raise RuntimeError(last_error_str)


# Constants
INFINITE = int(0xFFFFFFFF)
HISTORY_INDEX_POS = 0
HISTORY_INDEX_VEL = 1
MAX_AXES = 2
INVALID_LOG_CHANNEL = -1

class MemoryLogger:
    """
    Handles memory logging operations, including setting up the memory log,
    collecting log data, and managing log channels.
    """
    def __init__(self, error_queue=None, axis=0):
        self.log_data_history = [np.zeros((0,)), np.zeros((0,))]
        self.axis = axis
        self.error_queue = error_queue
        self.overflow_flag = 0
        
        # For collecting memory log, new wmx3_api and wmx3_log instances need to be created.	
        self.wmx3_api = WMX3Api()
        self.wmx3_api.CreateDevice('/opt/lmx', DeviceType.DeviceTypeNormal, INFINITE)
        self.wmx3_api.SetDeviceName('memory_log')

        self.wmx3_log = Log(self.wmx3_api)
        self.log_channel = self.check_memory_logchannel()        

    def __del__(self):
        """
        Destructor method to clean up resources when the instance is deleted.
        """
        if self.log_channel != INVALID_LOG_CHANNEL:
            self.close_log(self.log_channel)

        # Close wmx3_api.
        self.wmx3_api.CloseDevice()
            
    def is_available_logchannel(self, channel):    
        ret, mem_logstatus = self.wmx3_log.GetMemoryLogStatus(channel)    
        if ret != ErrorCode.PyNone:
            check_errorcode("GetMemoryLogStatus during is_available_logchannel", ret, self.error_queue)
            return False
        
        # REVISIT: The bufferOpened flag is alway TRUE even after closing the buffer.
        if mem_logstatus.logState != LogState.Idle and mem_logstatus.bufferOpened == True:
            print(f"mem_logstatus.LogState: {mem_logstatus.logState}, mem_logstatus.bufferOpened: {mem_logstatus.bufferOpened}")
            return False
        else:
            return True

    def add_error_queue(self, error_message):
        if self.error_queue:
            self.error_queue.put(error_message)

    def check_memory_logchannel(self):
        channel = INVALID_LOG_CHANNEL
        for current_channel in range(constants.maxLogChannel - 1, 1, -1):
            print(f"Trying memory log channel #{current_channel}")
            if self.is_available_logchannel(current_channel):
                ret = self.wmx3_log.OpenMemoryLogBuffer(current_channel);
                if ret == ErrorCode.PyNone:
                    channel = current_channel   

                    axis_sel = AxisSelection()
            
                    axis_sel.axisCount = MAX_AXES;
                    for idx in range(MAX_AXES):
                        axis_sel.SetAxis(idx, idx)

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
                    #     check_errorcode("SetEvent", ret, self.error_queue)            
                    # else:
                    #     memOption.triggerEventCount = 1
                    #     memOption.SetTriggerEventID(0, eventId)
                    #     wmxEventCtrl.EnableEvent(eventId, 1)
                
                    mem_option = MemoryLogOptions()
                    mem_option.triggerEventCount = 0

                    ret = self.wmx3_log.SetMemoryLog(channel, axis_sel, mem_option)
                    if ret != ErrorCode.PyNone:
                        check_errorcode("SetMemoryLog during check_memory_logchannel", ret, self.error_queue)
                        return INVALID_LOG_CHANNEL
                    
                    ret = self.wmx3_log.StartMemoryLog(channel)
                    if ret != ErrorCode.PyNone:
                        check_errorcode("StartMemoryLog during check_memory_logchannel", ret, self.error_queue)
                        return INVALID_LOG_CHANNEL

                    break
                else:
                    check_errorcode("OpenMemoryLogBuffer", ret, self.error_queue)              

        return channel

    def collect_logdata(self, channel, axis=0):
        updated_logdata = None

        ret, memory_logdata = self.wmx3_log.GetMemoryLogData(channel)
        if ret != ErrorCode.PyNone:
            check_errorcode("GetMemoryLogData during collect_logdata", ret, self.error_queue)
        else:
            if memory_logdata.overflowFlag > 0:
                print(f'(WARNING) Log overflow detected!')                                            
                overflow_flag += 1
            
            updated_logdata = [None]*memory_logdata.count                 

            if memory_logdata.count > 0:
                for index in range(memory_logdata.count):
                    logdata = memory_logdata.GetLogData(index)                                      
                    updated_logdata[index] = logdata.GetLogAxisData(axis)        
        
        return updated_logdata        
    
    def add_log_data(self, pos_logs, vel_logs):        
        self.log_data_history[HISTORY_INDEX_POS] = np.concatenate(
            (self.log_data_history[HISTORY_INDEX_POS], pos_logs), axis=0)
        self.log_data_history[HISTORY_INDEX_VEL] = np.concatenate(
            (self.log_data_history[HISTORY_INDEX_VEL], vel_logs), axis=0)

    def close_log(self, channel):
        ret, mem_logstatus = self.wmx3_log.GetMemoryLogStatus(channel)
        if ret != ErrorCode.PyNone:
            check_errorcode("GetMemoryLogStatus", ret, self.error_queue)
            return False

        if mem_logstatus.logState == LogState.Running:           
            ret = self.wmx3_log.StopMemoryLog(channel)
            if ret != ErrorCode.PyNone:
                check_errorcode("StopMemoryLog", ret, self.error_queue)
                return False
        
        sleep(0.1) # Wait for a while to finish the previous log operation

        ret = self.wmx3_log.CloseMemoryLogBuffer(channel)
        if ret != ErrorCode.PyNone:
            check_errorcode("CloseMemoryLogBuffer", ret, self.error_queue)
            return False
        
        self.log_channel = INVALID_LOG_CHANNEL

        return True

class WMX3LogManager:
    def __init__(self):
        self.log_data_history = [np.zeros((0,)), np.zeros((0,))]
        self.overflow_flag = 0

        # Initialize multiprocessing manager and shared resources
        self.manager = multiprocessing.Manager()
        self.stop_event = multiprocessing.Event()
        self.start_event = multiprocessing.Event()  # Event to signal subprocess start
        self.error_queue = self.manager.Queue()
        self.log_updater_result = self.manager.dict()

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

    def update_log_task(self, error_queue=None):
        """Worker subprocess to collect log data."""
        mem_logger = MemoryLogger(error_queue)

        try:
            # Signal that the subprocess has started
            self.start_event.set()

            while not self.stop_event.is_set():
                updated_logdata = mem_logger.collect_logdata(mem_logger.log_channel)

                if updated_logdata and len(updated_logdata) > 0:
                    pos_value_array = [data.feedbackPos for data in updated_logdata]
                    vel_value_array = [data.feedbackVelocity for data in updated_logdata]

                    # ret, mem_logstatus = mem_logger.wmx3_log.GetMemoryLogStatus(mem_logger.log_channel)
                    # if ret == ErrorCode.PyNone:
                    #     mem_logger.add_error_queue(f"mem_logstatus: {mem_logstatus.logState}, {mem_logstatus.samplesToCollect}, {mem_logstatus.samplesCollected}, {mem_logstatus.usageRate}")
                    
                    mem_logger.add_log_data(pos_value_array, vel_value_array)
                
                # Wait for data to be saved to the log buffer
                sleep(0.1)

            # Update the shared dictionary with results
            self.log_updater_result['overflow'] = mem_logger.overflow_flag
            self.log_updater_result['history'] = mem_logger.log_data_history

            print(f'[MemoryLogger Log] Count: {mem_logger.log_data_history[HISTORY_INDEX_POS].size}, Overflow: {mem_logger.overflow_flag}')

        except Exception as e:
            mem_logger.add_error_queue(str(e))

    def start_log(self):
        """Start the worker subprocess."""
        # Clear the stop_event and start_event before starting a worker subprocess
        self.stop_event.clear()
        self.start_event.clear()  # Clear the start_event
        self.overflow_flag = 0
        self.log_data_history = [np.zeros((0,)), np.zeros((0,))]

        # Start the worker subprocess
        self.log_update_process = Process(
            target=self.update_log_task, args=(self.error_queue,)
        )
        self.log_update_process.start()

        # Wait for the subprocess to signal that it has started
        if not self.start_event.wait(timeout=5):  # Timeout after 5 seconds
            print("Error: Subprocess failed to start within the timeout period.")
            return False

        print("start_log has executed successfully.")

        return True

    def stop_log(self):
        """Pause the worker subprocess and retrieve results."""
        # Signal the subprocess to stop
        self.stop_event.set()
        
        # Wait for the subprocess to finish
        if self.log_update_process and self.log_update_process.is_alive():
            self.log_update_process.join()

        # Check for errors in the subprocess
        if not self.error_queue.empty():
            error_message = self.error_queue.get()
            print(f"Messages in the log process: {error_message}")
            return

        # Get the updated log data
        self.overflow_flag = self.log_updater_result.get('overflow', 0)
        self.log_data_history = self.log_updater_result.get('history', [np.zeros((0,)), np.zeros((0,))])

        # Print the summary of updated logdata
        print(f'[Received Log] Count: {self.log_data_history[HISTORY_INDEX_POS].size}, Overflow: {self.overflow_flag}')


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