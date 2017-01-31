﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading;
using Windows.Devices.I2c;
using Windows.Devices.Spi;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using System.Diagnostics;
using Windows.System.Threading;
using Windows.ApplicationModel.Background;
using Windows.Storage;
using Windows.Storage.Search;
using System.Threading.Tasks;
using CanTest;
using Windows.UI;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace App1
{
    public sealed partial class MainPage : Page
    {
        // TODO: Add something to shutdown everything (with stopAllOperations flag)
        // TODO: Change sensor data to acquire only the constant g values -> low pass filter
        private MCP2515 mcp2515;
        private Timer stateTimer, errorTimer;
        private const byte SPI_CHIP_SELECT_LINE = 0;
        private int DELTA_T_TIMER_CALLBACK = 5, DELTA_T_MCP_EXECUTOR = 100, DELTA_T_ERROR_TIMER = 10;
        private const double encoderToDeg = ((double)360 / 34608);

        // DATA FOR ADXL SENSOR
        private const byte ACCEL_REG_X = 0x32;              /* Address of the X Axis data register                  */
        private const byte ACCEL_REG_Y = 0x34;              /* Address of the Y Axis data register                  */
        private const byte ACCEL_REG_Z = 0x36;              /* Address of the Z Axis data register                  */
        private const byte ACCEL_I2C_ADDR = 0x53;           /* 7-bit I2C address of the ADXL345 with SDO pulled low */
        private const byte ACCEL_SPI_RW_BIT = 0x80;         /* Bit used in SPI transactions to indicate read/write  */
        private const byte ACCEL_SPI_MB_BIT = 0x40;         /* Bit used to indicate multi-byte SPI transactions     */
        private const int ACCEL_RES = 1024;         /* The ADXL345 has 10 bit resolution giving 1024 unique values                     */
        private const int ACCEL_DYN_RANGE_G = 8;    /* The ADXL345 had a total dynamic range of 8G, since we're configuring it to +-4G */
        private const int UNITS_PER_G = ACCEL_RES / ACCEL_DYN_RANGE_G;  // Ratio of raw int values to G units          
        private short pwmValueTemp;
        private byte[] bytesToSend = new byte[3];

        struct McpExecutorDataFrame
        {
            public double pwmValue;
            public double encoderValue;
            public int ident;
            public float timeStamp;
        };

        // Data to set the execution context for checking the message answer from remote mcpExecutor when sending stop / start sequence
        enum CheckExecution
        {
            stopExecution,
            startExecution
        };

        enum HmiElementsStates
        {
            disableAll,
            enableAll,
            startIsPressed,
            stoppIsPressed,
            checkBoxIsSelected
        };

        private GlobalDataSet globalDataSet;
        private ServerComm serverComm;
        private Diagnose diagnose;
        private Task task_mcpExecutorService;
        private HmiElementsStates hmiElementsStates;

        private Pulses pulses;
        private const byte MAX_MCP_DEVICE_COUNTER = 2; // max. 255 
        private int MAX_WAIT_TIME = 5000; // milliseconds 
        private GpioPin[] mcpExecutor_request = new GpioPin[MAX_MCP_DEVICE_COUNTER];
        private GpioPin[] mcpExecutor_handshake = new GpioPin[MAX_MCP_DEVICE_COUNTER];
        private int mcpExecutorCounter;

        // DATA FOR ERROR HANDLING
        private const int MAX_ERROR_COUNTER_TRANSFER = 20;
        private int errorCounterTransfer;

        // DATA FOR DEBUGGING
        private Stopwatch timeStopper = new Stopwatch();
        private Stopwatch timer_programExecution = new Stopwatch();
        private Stopwatch timer_maxWaitTime = new Stopwatch();
        private Stopwatch timer_delay = new Stopwatch();
        private bool firstStart;
        private bool startButtonIsTrue = false;
        private bool stopSequenceIsActive;
        private bool getProgramDuration;
        private long timerValue;
        private long[] timerArray = new long[10];

        public MainPage()
        {
            this.InitializeComponent();

            // Initilize data
            errorCounterTransfer = 0;
            mcpExecutorCounter = 0;
            firstStart = true;
            stopSequenceIsActive = false;
            globalDataSet = new GlobalDataSet(); // Get things like mcp2515, logic_Mcp2515_Sender, logic_Mcp2515_Receiver
            serverComm = new ServerComm(globalDataSet);
            diagnose = new Diagnose(globalDataSet);
            mcp2515 = globalDataSet.Mcp2515;
            pulses = new Pulses();


            // Set active pulse as pre condition           
            changeActivePulse(Pulses.Pulse_Types.sinus);

            // USER CONFIGURATION
            globalDataSet.DebugMode = false;
            getProgramDuration = false;

            // Inititalize raspberry pi and gpio
            init_raspberry_pi_gpio();
            init_raspberry_pi_spi();

            // Inititalize mcp2515
            Task task_initMcp2515 = new Task(globalDataSet.init_mcp2515_task);
            task_initMcp2515.Start();
            task_initMcp2515.Wait();

            // Start executor service
            task_mcpExecutorService = new Task(mcpExecutorService_task);
            task_mcpExecutorService.Start();

            // Inititalize background tasks
            stateTimer = new Timer(this.StateTimer, null, 0, DELTA_T_TIMER_CALLBACK); // Create timer to display the state of message transmission
            errorTimer = new Timer(this.ErrorTimer, null, 0, DELTA_T_ERROR_TIMER); // Create timer to display the state of message transmission

            // Inititalize server
            Task<bool> serverStarted = serverComm.StartServer();
        }



        private void init_raspberry_pi_gpio()
        {
            if (globalDataSet.DebugMode) Debug.Write("Start GPIO init \n");

            var gpioController = GpioController.GetDefault();

            if (gpioController == null)
            {
                return;
            }
            try
            {
                if (globalDataSet.DebugMode) Debug.Write("Configure pins \n");
                // Configure pins
                globalDataSet.do_mcp2515_cs_sen = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_19, GpioPinValue.High, GpioPinDriveMode.Output);
                globalDataSet.di_mcp2515_int_sen = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_5, GpioPinDriveMode.Input);
                globalDataSet.do_mcp2515_cs_rec = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_12, GpioPinValue.High, GpioPinDriveMode.Output);
                globalDataSet.di_mcp2515_int_rec = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_13, GpioPinDriveMode.Input);
                globalDataSet.do_startAcquisition = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_17, GpioPinValue.Low, GpioPinDriveMode.Output);
            }
            catch (FileLoadException ex)
            {
                if (globalDataSet.DebugMode) Debug.Write("Exception in initGPIO: " + ex + "\n");
            }
        }

        private async void init_raspberry_pi_spi()
        {
            if (globalDataSet.DebugMode) Debug.Write("Init SPI interface" + "\n");
            try
            {
                var settings = new SpiConnectionSettings(SPI_CHIP_SELECT_LINE);
                settings.ClockFrequency = 5000000;
                settings.Mode = SpiMode.Mode3;
                string aqs = SpiDevice.GetDeviceSelector();
                var dis = await DeviceInformation.FindAllAsync(aqs);
                globalDataSet.SPIDEVICE = await SpiDevice.FromIdAsync(dis[0].Id, settings);
                if (globalDataSet.SPIDEVICE == null)
                {
                    if (globalDataSet.DebugMode) Debug.Write("SPI Controller is currently in use by another application. \n");
                    return;
                }
            }
            catch (Exception ex)
            {
                if (globalDataSet.DebugMode) Debug.Write("SPI Initialization failed. Exception: " + ex.Message + "\n");
                return;
            }

            // Send something to check that spi device is ready
            globalDataSet.Spi_not_initialized = true;
            while (globalDataSet.Spi_not_initialized)
            {
                bool error = false;
                try
                {
                    globalDataSet.SPIDEVICE.Write(new byte[] { 0xFF });
                }
                catch (Exception)
                {
                    error = true;
                }
                if (!error)
                {
                    globalDataSet.Spi_not_initialized = false;
                    if (globalDataSet.DebugMode) Debug.Write("Spi device ready" + "\n");
                }
                else
                {
                    if (globalDataSet.DebugMode) Debug.Write("Spi device not ready" + "\n");
                }
            }
        }

        private GpioPin configureGpio(GpioController gpioController, int gpioId, GpioPinDriveMode pinDriveMode)
        {
            GpioPin pinTemp;

            pinTemp = gpioController.OpenPin(gpioId);
            pinTemp.SetDriveMode(pinDriveMode);

            return pinTemp;
        }

        private GpioPin configureGpio(GpioController gpioController, int gpioId, GpioPinValue pinValue, GpioPinDriveMode pinDriveMode)
        {
            GpioPin pinTemp;

            pinTemp = gpioController.OpenPin(gpioId);
            pinTemp.Write(pinValue);
            pinTemp.SetDriveMode(pinDriveMode);

            return pinTemp;
        }

        private void StateTimer(object state)
        {
            bool indicatorMode = false;

            if (globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.Low)
            {
                indicatorMode = true;
            }
            else
            {
                indicatorMode = false;
            }

            /* UI updates must be invoked on the UI thread */
            var task = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                {
                    if (indicatorMode)
                    {
                        indicator.Background = new SolidColorBrush(Colors.Green);
                    }
                    else
                    {
                        indicator.Background = new SolidColorBrush(Colors.Red);
                    }

                });
        }

        private void ErrorTimer(object state)
        {
            // TODO Show red blinking warning message on screen
            if (errorCounterTransfer >= MAX_ERROR_COUNTER_TRANSFER)
            {
                if (globalDataSet.DebugMode) Debug.Write("ERROR TRANSFER - STOP ALL OPERATIONS" + "\n");
                globalDataSet.StopAllOperations = true;
                errorCounterTransfer = 0;
            }
        }

        private byte[] generateIdentifier(int identifierTemp)
        {
            byte[] identifier = BitConverter.GetBytes(identifierTemp);
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(identifier);
                if (globalDataSet.DebugMode) Debug.Write("IsLittleEndian \n");
            }

            if (globalDataSet.DebugMode) Debug.Write("Convert " + identifierTemp + " to " + identifier[0] + " and " + identifier[1] + "\n");
            if (globalDataSet.DebugMode) Debug.Write("Convert " + identifierTemp + " to " + identifier + "\n");

            // Return max 2 bytes
            return identifier;
        }

        private void SendMotorData(byte rxStateIst, byte rxStateSoll)
        {
            byte[] returnMessage = new byte[mcp2515.MessageSizeAdxl];
            byte[] returnMessageTemp = new byte[1];

            if ((rxStateIst & rxStateSoll) == 1)
            {
                byte[] spiMessage = new byte[1];

                //for (int i = 0; i < mcp2515.MessageSizeAdxl; i++) returnMessage[i] = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB0Dx[i]);
                returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER0);
                // We need to check sidl only because we have not so much devices.
                //Debug.WriteLine("REGISTER_RXB0SIDL: " + globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB0SIDL));
            }
            else if ((rxStateIst & rxStateSoll) == 2)
            {
                //for (int i = 0; i < mcp2515.MessageSizeAdxl; i++) returnMessage[i] = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB1Dx[i]);
                returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER1);
                //Debug.WriteLine("REGISTER_RXB1SIDL: " + globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB1SIDL));
            }
        }

        private McpExecutorDataFrame ReceiveEncoderData(byte rxStateIst, byte rxStateSoll)
        {
            byte[] returnMessage = new byte[mcp2515.MessageSizeAdxl];
            byte[] returnMessageTemp = new byte[1];

            if ((rxStateIst & rxStateSoll) == 1)
            {
                byte[] spiMessage = new byte[1];

                //for (int i = 0; i < mcp2515.MessageSizeAdxl; i++) returnMessage[i] = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB0Dx[i]);
                returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER0);
                // We need to check sidl only because we have not so much devices.
                //Debug.WriteLine("REGISTER_RXB0SIDL: " + globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB0SIDL));
            }
            else if ((rxStateIst & rxStateSoll) == 2)
            {
                //for (int i = 0; i < mcp2515.MessageSizeAdxl; i++) returnMessage[i] = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB1Dx[i]);
                returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER1);
                //Debug.WriteLine("REGISTER_RXB1SIDL: " + globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB1SIDL));
            }

            if (getProgramDuration) timerArray[3] = timer_programExecution.ElapsedMilliseconds;

            // Reset interrupt for buffer 0 because message is read -> Reset all interrupts
            //globalDataSet.mcp2515_execute_write_command(new byte[] { mcp2515.CONTROL_REGISTER_CANINTF, mcp2515.CONTROL_REGISTER_CANINTF_VALUE.RESET_ALL_IF }, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            //Debug.WriteLine("ret_o: " + returnMessage[0]);
            //Debug.WriteLine(returnMessage[1]);
            //Debug.WriteLine("ret_2: " + returnMessage[2]);

            // Read encoder value
            //int encoderDirection = returnMessage[0];
            //short encoderValue = BitConverter.ToInt16(returnMessage, 1);

            // Insert data to DataFrame
            McpExecutorDataFrame mcpExecutorDataFrame = new McpExecutorDataFrame();

            //if (encoderDirection == 0) mcpExecutorDataFrame.encoderValue = encoderValue * (-1);
            //else mcpExecutorDataFrame.encoderValue = encoderValue;

            //// Convert to an angle
            //mcpExecutorDataFrame.encoderValue = (mcpExecutorDataFrame.encoderValue * encoderToDeg);

            //if (globalDataSet.DebugMode) Debug.WriteLine("ENCODER: " + mcpExecutorDataFrame.encoderValue);

            ////mcpExecutorDataFrame.ident = signal_Id;
            ////mcpExecutorDataFrame.timeStamp = (float)timeStamp / 1000;

            //if (getProgramDuration) timerArray[4] = timer_programExecution.ElapsedMilliseconds;

            return mcpExecutorDataFrame;
        }

        private void MainPage_Unloaded(object sender, object args)
        {
            globalDataSet.SPIDEVICE.Dispose();
        }

        private void button_start_Click(object sender, RoutedEventArgs e)
        {
            startButtonIsTrue = true;
            changeHmiElements(HmiElementsStates.startIsPressed);
        }

        private void button_stopp_Click(object sender, RoutedEventArgs e)
        {
            startButtonIsTrue = false;
            changeHmiElements(HmiElementsStates.stoppIsPressed);
        }

        public async void mcpExecutorService_task()
        {
            await Task.Run(() => execServ_mcp2515());
        }

        private void checkBox_sinus_Checked(object sender, RoutedEventArgs e)
        {
            pulses.active_pulse_type = Pulses.Pulse_Types.sinus;
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
        }

        private void checkBox_sawtooth_Checked(object sender, RoutedEventArgs e)
        {
            pulses.active_pulse_type = Pulses.Pulse_Types.sawtooth;
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
        }

        private void checkBox_square_Checked(object sender, RoutedEventArgs e)
        {
            pulses.active_pulse_type = Pulses.Pulse_Types.square;
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
        }

        private void changeHmiElements(HmiElementsStates state)
        {
            // Change hmi elements
            switch (state)
            {
                case HmiElementsStates.disableAll:
                    button_start.IsEnabled = false;
                    button_stopp.IsEnabled = false;
                    checkBox_sinus.IsEnabled = false;
                    checkBox_sawtooth.IsEnabled = false;
                    checkBox_square.IsEnabled = false;
                    break;
                case HmiElementsStates.enableAll:
                    button_start.IsEnabled = true;
                    button_stopp.IsEnabled = true;
                    checkBox_sinus.IsEnabled = true;
                    checkBox_sawtooth.IsEnabled = true;
                    checkBox_square.IsEnabled = true;
                    break;
                case HmiElementsStates.startIsPressed:
                    button_start.IsEnabled = false;
                    button_stopp.IsEnabled = true;
                    checkBox_sinus.IsEnabled = false;
                    checkBox_sawtooth.IsEnabled = false;
                    checkBox_square.IsEnabled = false;
                    break;
                case HmiElementsStates.stoppIsPressed:
                    button_start.IsEnabled = true;
                    button_stopp.IsEnabled = false;
                    checkBox_sinus.IsEnabled = true;
                    checkBox_sawtooth.IsEnabled = true;
                    checkBox_square.IsEnabled = true;
                    break;
                case HmiElementsStates.checkBoxIsSelected:
                    if (checkBox_sinus.IsChecked == true)
                    {
                        checkBox_sawtooth.IsChecked = false;
                        checkBox_square.IsChecked = false;
                    }
                    else if (checkBox_sawtooth.IsChecked == true)
                    {
                        checkBox_sinus.IsChecked = false;
                        checkBox_square.IsChecked = false;
                    }
                    else
                    {
                        checkBox_sinus.IsChecked = false;
                        checkBox_sawtooth.IsChecked = false;
                    }
                    ;
                    break;
                default:
                    break;
            }
        }

        private void changeActivePulse(Pulses.Pulse_Types pulseType)
        {
            switch (pulseType)
            {
                case Pulses.Pulse_Types.sinus:
                    checkBox_sinus.IsChecked = true;
                    break;
                case Pulses.Pulse_Types.square:
                    checkBox_square.IsChecked = true;
                    break;
                case Pulses.Pulse_Types.sawtooth:
                    checkBox_sawtooth.IsChecked = true;
                    break;
                case Pulses.Pulse_Types.counter:
                    break;
                default:
                    break;                    
            }
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
            pulses.active_pulse_type = pulseType;
        }

        private int[] getActivePulseData()
        {
            switch (pulses.active_pulse_type)
            {
                case Pulses.Pulse_Types.sinus:
                    return pulses.Pulse_sinus;
                case Pulses.Pulse_Types.square:
                    return pulses.Pulse_square;
                case Pulses.Pulse_Types.sawtooth:
                    return pulses.Pulse_sawtooth;
                case Pulses.Pulse_Types.counter:
                    return pulses.Pulse_counter;
                default:
                    return pulses.Pulse_sinus;
            }
        }

        private void execServ_mcp2515()
        {
            long startTimeCHeck = 0;
            bool preCondIsSet = false;

            while (!globalDataSet.StopAllOperations)
            {
                // Wait until a client is connected and the spi device is ready to use
                // After this pre condition we are able to start the measurment via the HMI

                //if (globalDataSet.clientIsConnected & !globalDataSet.Spi_not_initialized & !preCondIsSet)
                if (!globalDataSet.Spi_not_initialized & !preCondIsSet)
                {
                    /* UI updates must be invoked on the UI thread */
                    var task = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                    {
                        changeHmiElements(HmiElementsStates.enableAll);
                    });
                    preCondIsSet = true;
                }

                if (!timer_delay.IsRunning)
                {
                    timer_delay.Reset();
                    timer_delay.Start();
                }

                // Send pulse to motor and receive encoder values
                // Start when the start button is pressed
                int[] pulseData = getActivePulseData();

                for (int i = 0; ((i < pulseData.Length) & (startButtonIsTrue)); i++)
                {
                    // Convert pulse to byte
                    if (pulseData[i] < 0)
                    {
                        pwmValueTemp = (short)(pulseData[i] * (-1));
                        bytesToSend[0] = (byte)0;
                    }
                    else
                    {
                        pwmValueTemp = (short)(pulseData[i]);
                        bytesToSend[0] = (byte)1;
                    }

                    //Debug.WriteLine(pwmValueTemp);

                    byte[] pwmValue_converted = BitConverter.GetBytes(pwmValueTemp);
                    if (BitConverter.IsLittleEndian) Array.Reverse(pwmValue_converted);
                    bytesToSend[1] = pwmValue_converted[0];
                    bytesToSend[2] = pwmValue_converted[1];


                    for (int j = 0; j < mcp2515.MessageSizePwm; j++) globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_load_tx_buffer0(bytesToSend[j], j, mcp2515.MessageSizePwm);
                    globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_execute_rts_command(0);

                    // Wait some time
                    startTimeCHeck = timer_delay.ElapsedMilliseconds;
                    while ((timer_delay.ElapsedMilliseconds - startTimeCHeck) <= 20) { }

                    executeAqcuisition();

                    // Wait some time
                    startTimeCHeck = timer_delay.ElapsedMilliseconds;
                    while ((timer_delay.ElapsedMilliseconds - startTimeCHeck) <= 20) { }
                }
            }
        }

        private void executeAqcuisition()
        {
            if (getProgramDuration) timerArray[0] = timer_programExecution.ElapsedMilliseconds;

            string pwmValue, encoderValue, zText, signal_Id, timeStamp;
            byte rxStateIst = 0x00;
            byte rxStateSoll = 0x03;

            if (globalDataSet.DebugMode) Debug.WriteLine("Wait until a message is received in buffer 0 or 1");

            // Wait until a message is received in buffer 0 or 1
            timer_maxWaitTime.Reset();
            timer_maxWaitTime.Start();
            //while ((globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.High) && timer_maxWaitTime.ElapsedMilliseconds <= MAX_WAIT_TIME)
            while ((globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.High))
            {
            }
            timer_maxWaitTime.Stop();

            //if (timer_maxWaitTime.ElapsedMilliseconds < MAX_WAIT_TIME)
            //{
            if (getProgramDuration) timerArray[1] = timer_programExecution.ElapsedMilliseconds;

            if (globalDataSet.DebugMode) Debug.WriteLine("Finished waiting, check which rx buffer.");
            // Check in which rx buffer the message is
            rxStateIst = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_get_state_command();

            if (globalDataSet.DebugMode) Debug.WriteLine("Read sensor values from device ");

            if (getProgramDuration) timerArray[2] = timer_programExecution.ElapsedMilliseconds;

            // Read the sensor data
            McpExecutorDataFrame mcpExecutorDataFrame = ReceiveEncoderData(rxStateIst, rxStateSoll);

            //// Create string with sensor content
            //pwmValue = String.Format("x{0:F3}", mcpExecutorDataFrame.pwmValue);
            //encoderValue = String.Format("y{0:F3}", mcpExecutorDataFrame.encoderValue);
            //zText = String.Format("z{0:F3}", 1);
            //signal_Id = mcpExecutorDataFrame.ident.ToString();
            //timeStamp = mcpExecutorDataFrame.timeStamp.ToString();

            //string message = pwmValue + "::" + encoderValue + "::" + zText + "::" + timeStamp;
            //diagnose.sendToSocket(signal_Id, message);

            ////Debug.WriteLine("MESSAGE: " + message);

            //if (globalDataSet.DebugMode) Debug.WriteLine("sensorId: " + signal_Id);
            //if (globalDataSet.DebugMode) Debug.WriteLine("message: " + message);

            //if (getProgramDuration) timerArray[5] = timer_programExecution.ElapsedMilliseconds;

            //// Reset interrupt for buffer 0 because message is read -> Reset all interrupts
            ////globalDataSet.mcp2515_execute_write_command(new byte[] { mcp2515.CONTROL_REGISTER_CANINTF, mcp2515.CONTROL_REGISTER_CANINTF_VALUE.RESET_ALL_IF }, globalDataSet.MCP2515_PIN_CS_RECEIVER);

            //if (getProgramDuration) timerArray[6] = timer_programExecution.ElapsedMilliseconds;
            //if (getProgramDuration) for (int i = 0; i < timerArray.Length; i++) Debug.WriteLine(timerArray[i]);
            //}
        }

    }
}


