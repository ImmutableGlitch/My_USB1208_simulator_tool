using System;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using MccDaq;
using AnalogIO;
using DigitalIO;
using EventSupport;
using ErrorDefs;
using System.Threading;
using System.Collections.Generic;
using System.IO;

namespace Sim
{
    public struct TUserData
    {
        /// <summary>
        ///	  To use member functions of the Form as MccDaq event handlers, we
        ///    must declare them as static member functions. However, static 
        ///    member functions are not associated with any class instance. 
        ///    So, we send a reference to 'this' class instance through the 
        ///    UserData parameter; but classes are reference types, which cannot
        ///    be directly converted to pointers to be passed in as UserData. 
        ///    Instead, we must wrap the class reference in a value type, such as
        ///    a 'struct', before converting to a pointer. TUserData is a value
        ///    type structure that simply wraps the class reference.
        /// </summary>
        /// 
        public object ThisObj;
    }

    public class frmEventDisplay : Form
    {

        #region Global Variables

        private System.Windows.Forms.Timer measurementTimer;

        // Create a new MccBoard object for Board 0
        MccBoard DaqBoard = new MccBoard(0);

        private int NumAIChans, NumEvents;
        public int ADResolution;

        public static int DesiredRate = 1000;
        // if desired rate higher than 1000 then = ((NumSamplesForAllChannels / (DesiredRate / 1000)));

        public static int NumSamplesForAllChannels = 240;
        public static float Cutoff          = 0.1f;

        public Range Range = Range.Bip10Volts;
        public ScanOptions Options = ScanOptions.Background |
                                     ScanOptions.ExtTrigger;
        public TUserData _userData;
        public IntPtr _ptrUserData;
        public IntPtr MemHandle;
        public EventCallback _ptrMyCallback;
        public EventCallback _ptrOnScanError;

        clsAnalogIO AIOProps = new clsAnalogIO();
        clsEventSupport SupportedEvents = new clsEventSupport();
        clsDigitalIO DioProps = new clsDigitalIO();
        ErrorInfo digitalStatus;

        public static List<ushort>[] channels;
        public static ushort[] rawDataCopy;

        // Digital Vars
        int NumPorts, NumBits, FirstBit;
        int PortType, ProgAbility;

        DigitalPortType PortNum;
        DigitalPortDirection Direction;

        private enum Pins
        {
            ClockDirection = 1,
            Clock          = 2,
            StartMeasure   = 7
        }

        private enum ChipSelectPins
        {
            LaserOnOff        = 0,
            LEDPowerFine      = 3,
            LEDPowerCoarse    = 4,
            PulseFine         = 5,
            PulseCoarse       = 6,
            DataPMTGainFine   = 8,
            DataPMTGainCoarse = 9,
            DataPMTOffset     = 10,
            LEDPMTGain        = 11
        }

        static bool logging;
        bool settingsChanged = false;

        bool autotune             = false;
        float desiredBrightness   = 1.7f;
        float brightnessTolerance = 0.02f;
        float coarseStep          = 0.18f;
        float fineStep            = 0.01f;
        
        #endregion

        #region ConfigVars

        // Digital Outputs
        public static ushort LED_power_fine     = 0;  // Number of steps for up direction?
        public static ushort LED_power_coarse   = 0;
        public static ushort PulseWidth_fine    = 0;
        public static ushort PulseWidth_coarse  = 0;
        public static ushort PMT_gain_fine      = 0;
        public static ushort PMT_gain_coarse    = 0;
        public static ushort PMT_offset         = 0;
        public static ushort LEDPMT_gain_fine   = 0;
        public static ushort LEDPMT_gain_coarse = 0;
        public static ushort LEDPMT_offset      = 0;

        // Analog Inputs
        public static ushort latest_LED_current_raw     = 0; // 0 to 4095
        public static ushort latest_LED_voltage_raw     = 0;
        public static ushort latest_LED_brightness_raw  = 0;
        public static ushort latest_LED_temperature_raw = 0;
        public static ushort latest_Pulse_raw           = 0;
        public static ushort latest_PMT_flu_raw         = 0;
        public static ushort latest_PMT_gain_raw        = 0;
        public static ushort latest_PMT_offset_raw      = 0;

        public static bool LED_ON;

        private Panel panel1;
        private Label label5;
        private Label label10;
        private Label label0;
        private Label label9;
        private NumericUpDown LEDpowerCoarse;
        private NumericUpDown PMTgainFine;
        private Label label6;
        private NumericUpDown PulseDurationFine;
        private NumericUpDown PulseDurationCoarse;
        private NumericUpDown LEDpowerFine;
        private NumericUpDown PMTgainCoarse;
        private Label label7;
        private Label PMT_Flu_label;
        private NumericUpDown PMToffset;
        private Label LED_Temperature_label;
        private Label label8;
        private Label LED_Brightness_label;
        private Label label11;
        private Label LED_Voltage_label;
        private Label label12;
        private Label LED_Current_label;
        private Label label13;
        private Label label14;
        public Button cmdStart;
        private Label PMT_offset_label;
        private Label label4;
        private Label PMT_gain_label;
        private Button btnSave;
        private Label awd;
        private CheckBox cbLog;
        private Button btnLoad;
        private NumericUpDown nudSampleRate;
        private Label label15;
        private NumericUpDown nudSampleAmount;
        private Label label2;
        private NumericUpDown nudCutoff;
        private Label label16;
        private Panel panel2;
        private CheckBox cbFlu;
        private CheckBox cbOffset;
        private CheckBox cbGain;
        private CheckBox cbTemperature;
        private CheckBox cbBrightness;
        private CheckBox cbCurrent;
        private CheckBox cbVoltage;
        private System.Windows.Forms.DataVisualization.Charting.Chart graph;
        private Button btnStop;
        private Button btnStart;
        private NumericUpDown nudInterval;
        private Label label1;
        private CheckBox cbAutotune;
        private Label label18;
        private Label label17;
        private NumericUpDown nudCoarseStep;
        private NumericUpDown nudFineStep;
        private GroupBox gbAutotune;
        private Label label19;
        private Panel panel3;
        private NumericUpDown nudDesiredBrightness;
        private Label label20;
        private NumericUpDown nudBrightnessTolerance;
        private NumericUpDown LEDgain;
        private Label label22;
        private Label label3;

        #endregion

        public delegate void DelegateCallback();

        private void frmEventDisplay_Load(object sender, EventArgs e)
        {
            int x = (Screen.PrimaryScreen.WorkingArea.Width - Width);
            int y = (Screen.PrimaryScreen.WorkingArea.Height - Height);
            Location = new System.Drawing.Point(x, y);

            int LowChan, EventMask;
            TriggerType TrigType;

            InitUL();

            // determine the number of analog channels and their capabilities
            int ChannelType = clsAnalogIO.ANALOGINPUT;
            NumAIChans = AIOProps.FindAnalogChansOfType(DaqBoard, ChannelType,
                out ADResolution, out Range, out LowChan, out TrigType);
            EventMask = clsEventSupport.DATAEVENT |
                clsEventSupport.ENDEVENT | clsEventSupport.ERREVENT;
            NumEvents = SupportedEvents.FindEventsOfType(DaqBoard, EventMask);

            if (NumAIChans == 0) // ERROR
            {
                MessageBox.Show("Board " + DaqBoard.BoardNum.ToString() + " does not have analog input channels.");
                cmdStart.Enabled = false;
                Close();
            }
            else if (!(EventMask == NumEvents)) // ERROR
            {
                MessageBox.Show("Board " + DaqBoard.BoardNum.ToString() + " doesn't support the specified event types.");
                cmdStart.Enabled = false;
                Close();
            }
            else
            {
                // Setup Digital port
                PortType = clsDigitalIO.PORTOUT;
                NumPorts = DioProps.FindPortsOfType(DaqBoard, PortType, out ProgAbility, out PortNum, out NumBits, out FirstBit);

                if (ProgAbility == clsDigitalIO.PROGPORT)
                {
                    Direction = DigitalPortDirection.DigitalOut;
                    digitalStatus = DaqBoard.DConfigPort(PortNum, Direction);
                }

                ResetResistors();
                SetupMemoryAllocation();
                DisableEvents();
                EnableEvents();
                InitTimer();
            }

        }

        private void frmEventDisplay_FormClosing(object sender, FormClosingEventArgs e)
        {

            if (NumAIChans > 0)
            {
                Marshal.FreeCoTaskMem(_ptrUserData);

                //finally, free the data buffer
                if (MemHandle != IntPtr.Zero)
                    MccService.WinBufFreeEx(MemHandle);
                MemHandle = IntPtr.Zero;

                // Stop any active background operations. No background 
                // operations can be active while disabling event handlers.
                DaqBoard.StopBackground(FunctionType.AiFunction);

                // Remove any active event handlers & free the unmanaged resouces
                DaqBoard.DisableEvent(EventType.OnEndOfInputScan);
            }

            ushort DataValue = 0;

            if (ProgAbility == clsDigitalIO.PROGPORT)
            {
                ErrorInfo ULStat = DaqBoard.DOut(PortNum, DataValue);

                Direction = DigitalPortDirection.DigitalIn;
                ULStat = DaqBoard.DConfigPort(PortNum, Direction);
            }

        }

        #region Events and memory management

        public void SetupMemoryAllocation()
        {
            MemHandle = MccService.WinBufAllocEx(NumSamplesForAllChannels);

            if (MemHandle == IntPtr.Zero) throw
                (new OutOfMemoryException("Insufficient memory."));

            //store a reference to this instance in the TUserData struct
            _userData.ThisObj = this;

            //get a pointer to the TUserData struct to pass to EnableEvent
            _ptrUserData = Marshal.AllocCoTaskMem(Marshal.SizeOf(_userData));
            Marshal.StructureToPtr(_userData, _ptrUserData, false);

            //get pointers to the event handlers to be called...
            _ptrMyCallback = new EventCallback(MyCallback);
            _ptrOnScanError = new EventCallback(OnScanError);
        }

        public void EnableEvents()
        {
            // OnEndOfInputScan
            uint eventSize = 0;
            EventType eventType = EventType.OnEndOfInputScan;
            ErrorInfo ULStat = DaqBoard.EnableEvent(eventType, eventSize, _ptrMyCallback, _ptrUserData);

            // OnScanError
            eventType = EventType.OnScanError;
            ULStat = DaqBoard.EnableEvent(eventType, 0, _ptrOnScanError, _ptrUserData);

            try
            {
                ULStat = DaqBoard.AInScan(0, 7, NumSamplesForAllChannels, ref DesiredRate, Range, MemHandle, Options);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message + Environment.NewLine
                          + ULStat.Message + Environment.NewLine + ULStat.Value);
            }
        }

        public void DisableEvents()
        {
            //Disable and disconnect all event types with MccBoard.DisableEvent()
            DaqBoard.StopBackground(FunctionType.AiFunction);
            DaqBoard.DisableEvent(EventType.AllEventTypes);
        }

        public static void OnScanError(int bd, EventType et, uint scanError, IntPtr pdata)
        {
            //OnScanError is a static member function. So, we must do some work to get the reference
            // to this object. Recall that we passed in a pointer to a struct that wrapped the 
            // class reference as UserData...
            TUserData thisStruct = (TUserData)Marshal.PtrToStructure(pdata, typeof(TUserData));
            frmEventDisplay ThisObj = (frmEventDisplay)thisStruct.ThisObj;

            ThisObj.DaqBoard.StopBackground(FunctionType.AiFunction);
        }

        #endregion

        #region callback and data processing

        public static void MyCallback(int bd, EventType et, uint sampleCount, IntPtr pUserData)
        {
            // MyCallback is a static member function. So, we must do some work to get the reference
            // to this object. Recall that we passed in a pointer to a struct that wrapped the 
            // class reference as UserData.

            try
            {
                if (et == EventType.OnEndOfInputScan)
                {
                    TUserData userStruct = (TUserData)Marshal.PtrToStructure(pUserData, typeof(TUserData));
                    frmEventDisplay ThisObj = (frmEventDisplay)userStruct.ThisObj;

                    ushort[] rawData = new ushort[NumSamplesForAllChannels];
                    MccService.WinBufToArray(ThisObj.MemHandle, rawData, 0, NumSamplesForAllChannels);
                    rawDataCopy = (ushort[])rawData.Clone();

                    // If the event is the end of acquisition, release the 
                    // resources in preparation for the next scan.
                    ThisObj.DaqBoard.StopBackground(FunctionType.AiFunction);
                    ThisObj.ScanComplete();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "MyCallback Method");
            }

        }

        public void ScanComplete()
        {
            try
            {
                if (InvokeRequired == true)
                    Invoke(new DelegateCallback(ScanComplete));
                else
                {
                    var channels = SplitArrayToChannels(rawDataCopy);
                    ProcessChannelData(channels);
                    if (logging) LogData();
                    graphData(channels);

                    if (autotune)
                        if (BrightnessValid()) { DisplayData(true); } else { DisplayData(false); }
                    else
                        DisplayData(true);


                    ErrorInfo ULStat = null;
                    try
                    {
                        ULStat = DaqBoard.AInScan(0, (int)Pins.StartMeasure, NumSamplesForAllChannels, ref DesiredRate, Range, MemHandle, Options);
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message + Environment.NewLine
                                  + ULStat.Message + Environment.NewLine + ULStat.Value);
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "ScanComplete Method");
            }
        }

        private bool BrightnessValid()
        {
            float f;
            DaqBoard.ToEngUnits(Range, latest_LED_brightness_raw, out f);

            // Truncate to 5 DP then remove from 5 volts
            //float value = 5.000f - (float)(Math.Truncate((float)f * 10000.0) / 10000.0);
            float value     = (float)nudDesiredBrightness.Value - 
                              (float)(Math.Truncate((float)f * 10000.0) / 10000.0);

            float coarseAmount   = 0f;
            float fineAmount     = 0f;

            try
            {
                if (value > brightnessTolerance) // need to increase power by value amount
                {
                    // if the difference is too large then use coarse steps
                    if (value > coarseStep)
                    {
                        coarseAmount = (int)Math.Round(value / coarseStep);
                        if (LEDpowerCoarse.Value + (int)coarseAmount > LEDpowerCoarse.Maximum)
                            LEDpowerCoarse.Value = LEDpowerCoarse.Maximum;
                        else
                            LEDpowerCoarse.Value += (int)coarseAmount;
                    }
                    
                    value -= coarseStep * (int)coarseAmount;
                    value = value < 0 ? 0 : value; // set to zero if negative

                    // use fine steps
                    fineAmount = (int)Math.Round(value / fineStep);
                    if (LEDpowerFine.Value + (int)fineAmount > LEDpowerFine.Maximum)
                        LEDpowerFine.Value = LEDpowerFine.Maximum;
                    else
                        LEDpowerFine.Value += (int)fineAmount;

                    return false;
                }
                else if (value < -brightnessTolerance) // need to decrease by value amount
                {
                    if (value < -coarseStep)
                    {
                        coarseAmount = (int)Math.Round(value / coarseStep);
                        if (LEDpowerCoarse.Value + (int)coarseAmount < LEDpowerCoarse.Minimum) // add the negative number
                            LEDpowerCoarse.Value = LEDpowerCoarse.Minimum;
                        else
                            LEDpowerCoarse.Value += (int)coarseAmount;
                    }

                    value -= coarseStep * (int)coarseAmount;
                    value = value > 0 ? 0 : value; // set to zero if positive

                    // use fine steps
                    fineAmount = (int)Math.Round(value / fineStep);
                    if (LEDpowerFine.Value + (int)fineAmount < LEDpowerFine.Minimum) // add the negative number
                        LEDpowerFine.Value = LEDpowerFine.Minimum;
                    else
                        LEDpowerFine.Value += (int)fineAmount;

                    return false;
                }
            }catch(Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

            return true;
        }

        public static List<ushort>[] SplitArrayToChannels(ushort[] array)
        {
            // Create and instantiate array which holds 8 lists
            channels = new List<ushort>[8];
            for (int i = 0; i < 8; i++)
            {
                channels[i] = new List<ushort>();
            }

            // Place array data into correct list
            ushort index = 0;
            for (int i = 0; i < array.Length; i++)
            {
                if (index > 7) index = 0;
                channels[index++].Add(array[i]);
            }

            return channels;
        }

        public static void ProcessChannelData(List<ushort>[] channels)
        {
            var channelSamples = NumSamplesForAllChannels / 8;
            var bad_data_cutoff = (ushort)(channelSamples * Cutoff);

            for (int x = 0; x < channels.Length; x++)
            {
                int average = 0;
                for (int i = bad_data_cutoff; i < channelSamples; i++)
                {
                    average += channels[x][i];
                }

                // Store average within correct variable
                average = average / (channelSamples - bad_data_cutoff);
                switch (x)
                {
                    case 0:
                        latest_LED_current_raw = (ushort)average;
                        break;

                    case 1:
                        latest_LED_temperature_raw = (ushort)average;
                        break;

                    case 2:
                        latest_Pulse_raw = (ushort)average;
                        break;

                    case 3:
                        latest_LED_voltage_raw = (ushort)average;
                        break;

                    case 4:
                        latest_PMT_gain_raw = (ushort)average;
                        break;

                    case 5:
                        latest_PMT_offset_raw = (ushort)average;
                        break;

                    case 6:
                        latest_PMT_flu_raw = (ushort)average;
                        break;

                    case 7:
                        latest_LED_brightness_raw = (ushort)average;
                        break;

                    default:
                        break;
                }
            }
        }

        private void DisplayData(bool display)
        {
            if (display)
            {
                float convertedData;

                DaqBoard.ToEngUnits(Range, latest_LED_current_raw, out convertedData);
                LED_Current_label.Text = ((convertedData * 127.06) - 4.7517).ToString("F4") + " mA";

                DaqBoard.ToEngUnits(Range, latest_LED_voltage_raw, out convertedData);
                LED_Voltage_label.Text = ((convertedData * 2.7953) - 1.1474).ToString("F4") + " V";

                DaqBoard.ToEngUnits(Range, latest_LED_brightness_raw, out convertedData);
                LED_Brightness_label.Text = convertedData.ToString("F4") + " V";

                DaqBoard.ToEngUnits(Range, latest_LED_temperature_raw, out convertedData);
                LED_Temperature_label.Text = convertedData.ToString("F4") + " V";

                DaqBoard.ToEngUnits(Range, latest_PMT_gain_raw, out convertedData);
                PMT_gain_label.Text = convertedData.ToString("F4") + " V";

                DaqBoard.ToEngUnits(Range, latest_PMT_offset_raw, out convertedData);
                PMT_offset_label.Text = convertedData.ToString("F4") + " V";

                DaqBoard.ToEngUnits(Range, latest_PMT_flu_raw, out convertedData);
                PMT_Flu_label.Text = convertedData.ToString("F4") + " V";
            }
            else
            {
                LED_Current_label.Text = "---";
                LED_Voltage_label.Text = "---";
                LED_Brightness_label.Text = "---";
                LED_Temperature_label.Text = "---";
                PMT_gain_label.Text = "---";
                PMT_offset_label.Text = "---";
                PMT_Flu_label.Text = "---";
            }
        }

        private void graphData(List<ushort>[] data)
        {
            graph.ChartAreas[0].AxisX.Minimum = 0;
            graph.ChartAreas[0].AxisX.Title = "Time (ms)";
            graph.ChartAreas[0].AxisX.MajorGrid.LineDashStyle = System.Windows.Forms.DataVisualization.Charting.ChartDashStyle.NotSet;
            graph.ChartAreas[0].AxisY.MajorGrid.LineDashStyle = System.Windows.Forms.DataVisualization.Charting.ChartDashStyle.Dash;
            graph.ChartAreas[0].AxisY.MajorGrid.LineColor = System.Drawing.Color.LightGray;
            graph.ChartAreas[0].AlignmentStyle = System.Windows.Forms.DataVisualization.Charting.AreaAlignmentStyles.AxesView;


            //graph.ChartAreas[0].AxisY.Title = cbVoltage.Checked ? "LED Voltage" : "Gain (V)"; //_UV_ label needs to be dynamic
            graph.Series.Clear();

            graph.Series.Add("PULSE");
            graph.Series["PULSE"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
            graph.Series["PULSE"].Color = System.Drawing.Color.Red;

            if (cbCurrent.Checked)
            {
                graph.Series.Add("LED_CURRENT");
                graph.Series["LED_CURRENT"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["LED_CURRENT"].Color = System.Drawing.Color.Green;
            }
            if (cbVoltage.Checked)
            {
                graph.Series.Add("LED_VOLTAGE");
                graph.Series["LED_VOLTAGE"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["LED_VOLTAGE"].Color = System.Drawing.Color.Yellow;
            }
            if (cbBrightness.Checked)
            {
                graph.Series.Add("BRIGHTNESS");
                graph.Series["BRIGHTNESS"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["BRIGHTNESS"].Color = System.Drawing.Color.Blue;
            }
            if (cbTemperature.Checked)
            {
                graph.Series.Add("TEMPERATURE");
                graph.Series["TEMPERATURE"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["TEMPERATURE"].Color = System.Drawing.Color.Orange;
            }
            if (cbGain.Checked)
            {
                graph.Series.Add("GAIN");
                graph.Series["GAIN"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["GAIN"].Color = System.Drawing.Color.MediumPurple;
            }
            if (cbOffset.Checked)
            {
                graph.Series.Add("OFFSET");
                graph.Series["OFFSET"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["OFFSET"].Color = System.Drawing.Color.Pink;
            }
            if (cbFlu.Checked)
            {
                graph.Series.Add("FLU");
                graph.Series["FLU"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                graph.Series["FLU"].Color = System.Drawing.Color.Cyan;
            }


            float convertedData;
            double newValue = 0;

            var numOfSamplesPerChannel = (NumSamplesForAllChannels / 8);
            for (int i = 0; i < numOfSamplesPerChannel; i++)
            {
                DaqBoard.ToEngUnits(Range, data[2][i], out convertedData);
                graph.Series["PULSE"].Points.AddXY(i, convertedData);

                if (cbCurrent.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[0][i], out convertedData);
                    graph.Series["LED_CURRENT"].Points.AddXY(i, convertedData);
                }
                if (cbVoltage.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[3][i], out convertedData);
                    newValue = (convertedData * 2.7953) - 1.1474;
                    graph.Series["LED_VOLTAGE"].Points.AddXY(i, newValue);
                }
                if (cbBrightness.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[7][i], out convertedData);
                    graph.Series["BRIGHTNESS"].Points.AddXY(i, convertedData);
                }
                if (cbTemperature.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[1][i], out convertedData);
                    graph.Series["TEMPERATURE"].Points.AddXY(i, convertedData);
                }
                if (cbGain.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[4][i], out convertedData);
                    graph.Series["GAIN"].Points.AddXY(i, convertedData);
                }
                if (cbOffset.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[5][i], out convertedData);
                    graph.Series["OFFSET"].Points.AddXY(i, convertedData);
                }
                if (cbFlu.Checked)
                {
                    DaqBoard.ToEngUnits(Range, data[6][i], out convertedData);
                    graph.Series["FLU"].Points.AddXY(i, convertedData);
                }
            }//for
        }

        private void LogData()
        {
            var location = Path.GetDirectoryName(Application.ExecutablePath);
            var dateToday = DateTime.Today.ToShortDateString().Replace('/', '_');
            string filename = string.Format(@"{0}\UV_LOG_{1}.csv", location, dateToday);
            bool existing = File.Exists(filename);

            try
            {
                using (TextWriter writer = new StreamWriter(filename, true, System.Text.Encoding.UTF8))
                {
                    if (!existing) // if this is the first time file is created then print header
                    {
                        // Print Header for raw data and settings
                        writer.Write(
                            "\n\nCurrent (mA),Temp,Voltage,Gain,Offset,Flu,Brightness,Time, , ,Power_coarse,Power_fine,"+
                            "Pulse_coarse,Pulse_fine,Gain_coarse,Gain_fine,Offset,Sample_Rate,"+
                            "Num_Samples,Cutoff,AutoTune,Desired_Brightness,Bright_Tolerance,Coarse_Step,Fine_Step\n");
                    }

                    // Print all samples excluding channel 2
                    float convertedData;

                    DaqBoard.ToEngUnits(Range, latest_LED_current_raw, out convertedData);
                    writer.Write(((convertedData * 127.06) - 4.7517).ToString("F4") + ",");

                    DaqBoard.ToEngUnits(Range, latest_LED_temperature_raw, out convertedData);
                    writer.Write(convertedData.ToString("F4") + ",");

                    DaqBoard.ToEngUnits(Range, latest_LED_voltage_raw, out convertedData);
                    writer.Write(((convertedData * 2.7953) - 1.1474).ToString("F4") + ",");

                    DaqBoard.ToEngUnits(Range, latest_PMT_gain_raw, out convertedData);
                    writer.Write(convertedData.ToString("F4") + ",");

                    DaqBoard.ToEngUnits(Range, latest_PMT_offset_raw, out convertedData);
                    writer.Write(convertedData.ToString("F4") + ",");

                    DaqBoard.ToEngUnits(Range, latest_PMT_flu_raw, out convertedData);
                    writer.Write(convertedData.ToString("F4") + ",");

                    DaqBoard.ToEngUnits(Range, latest_LED_brightness_raw, out convertedData);
                    writer.Write(convertedData.ToString("F4") + ",");

                    writer.Write(String.Format("{0:h:mm:ss}", DateTime.Now));

                    if (!existing || settingsChanged)
                    {
                        // Print Settings used on form
                        writer.Write(",,,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}",
						LEDpowerCoarse.Value, LEDpowerFine.Value, PulseDurationCoarse.Value, PulseDurationFine.Value,
						PMTgainCoarse.Value, PMTgainFine.Value, PMToffset.Value, DesiredRate, NumSamplesForAllChannels/8,
						nudCutoff.Value, autotune, desiredBrightness,brightnessTolerance,coarseStep,fineStep);
                    }

                    writer.Write("\n");
                    writer.Close();
                }
            }
            catch (IOException ex)
            {
                Console.WriteLine(ex.Message);
                MessageBox.Show("Can't write log if it's already open.");
            }

            settingsChanged = false;
        }
        #endregion

        #region IO Helper functions

        private void DBitOut(int bitNum, int nHighLow)
        {
            var portType = DigitalPortType.FirstPortA;
            var bitValue = (nHighLow > 0) ? DigitalLogicState.High : DigitalLogicState.Low;

            ErrorInfo status = DaqBoard.DBitOut(portType, bitNum, bitValue);
            if (status.Value != ErrorInfo.ErrorCode.NoErrors) Console.WriteLine(status.Message);
        }

        public void ResetResistors()
        {
            DeselectAllResistors();

            // Select these pins
            DBitOut((int)ChipSelectPins.LEDPowerFine,   0);
            DBitOut((int)ChipSelectPins.LEDPowerCoarse, 0);
            DBitOut((int)ChipSelectPins.PulseFine,      0);
            DBitOut((int)ChipSelectPins.PulseCoarse,    0);
            DBitOut((int)ChipSelectPins.LEDPMTGain,     0);

            // Direction set to high means DOWN for Pulse and Power
            DBitOut((int)Pins.ClockDirection, 1);

            //Iterate until 100 steps clocked
            int nSteps = 0;
            while (nSteps < 100)
            {
                ClockResistors();
                nSteps++;
            }

            DeselectAllResistors();

            // Select these pins
            DBitOut((int)ChipSelectPins.DataPMTGainFine,   0);
            DBitOut((int)ChipSelectPins.DataPMTGainCoarse, 0);
            DBitOut((int)ChipSelectPins.DataPMTOffset,     0);

            // Direction set to low means DOWN for Gain and Offset
            DBitOut((int)Pins.ClockDirection, 0);

            //Iterate until 100 steps clocked
            nSteps = 0;
            while (nSteps < 100)
            {
                ClockResistors();
                nSteps++;
            }
        }

        private void DeselectAllResistors()
        {
            foreach (int pin in Enum.GetValues(typeof(ChipSelectPins)))
            {
                DBitOut(pin, 1);
            }
            Thread.Sleep(10);
        }

        public void SetResistorValue(int pin, int currentValue, int newValue)
        {
            DeselectAllResistors();

            // Select specific pin
            DBitOut(pin, 0);

            int clockAmount;
            if (newValue > currentValue)
            {
                if (pin == (int)ChipSelectPins.DataPMTGainFine   || 
                    pin == (int)ChipSelectPins.DataPMTGainCoarse ||
                    pin == (int)ChipSelectPins.DataPMTOffset)

                    // Gain and offset direction are inverted
                    DBitOut((int)Pins.ClockDirection, 1);
                else
                    DBitOut((int)Pins.ClockDirection, 0);

                clockAmount = (newValue - currentValue);
            }
            else
            {
                if (pin == (int)ChipSelectPins.DataPMTGainFine   ||
                    pin == (int)ChipSelectPins.DataPMTGainCoarse ||
                    pin == (int)ChipSelectPins.DataPMTOffset)

                    // Gain and offset direction are inverted
                    DBitOut((int)Pins.ClockDirection, 0);
                else
                    DBitOut((int)Pins.ClockDirection, 1);

                clockAmount = (currentValue - newValue);
            }

            for (int i = 0; i < clockAmount; i++)
            {
                ClockResistors();
            }

            settingsChanged = true;
        }

        private void ClockResistors()
        {
            DBitOut((int)Pins.Clock, 1);
            Thread.Sleep(10);
            DBitOut((int)Pins.Clock, 0);
            Thread.Sleep(10);
            DBitOut((int)Pins.Clock, 1);
            Thread.Sleep(10);
        }

        #endregion

        #region form controls

        private void cmdStart_Click(object sender, System.EventArgs e)
        {
            DBitOut((int)Pins.StartMeasure, 1);
            Thread.Sleep(10);
            DBitOut((int)Pins.StartMeasure, 0);
            Thread.Sleep(10);
        }

        public void InitTimer()
        {
            measurementTimer = new System.Windows.Forms.Timer();
            measurementTimer.Tick += new EventHandler(Measure_Tick);
            measurementTimer.Interval = (int)nudInterval.Value * 1000; // in miliseconds
        }

        private void Measure_Tick(object sender, EventArgs e)
        {
            cmdStart_Click(null, null);
        }

        private void LEDpowerFine_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.LEDPowerFine, LED_power_fine, (int)((NumericUpDown)sender).Value);
            LED_power_fine = (ushort)((NumericUpDown)sender).Value;
        }

        private void LEDpowerCoarse_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.LEDPowerCoarse, LED_power_coarse, (int)((NumericUpDown)sender).Value);
            LED_power_coarse = (ushort)((NumericUpDown)sender).Value;
        }

        private void PulseDurationFine_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.PulseFine, PulseWidth_fine, (int)((NumericUpDown)sender).Value);
            PulseWidth_fine = (ushort)((NumericUpDown)sender).Value;
        }

        private void PulseDurationCoarse_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.PulseCoarse, PulseWidth_coarse, (int)((NumericUpDown)sender).Value);
            PulseWidth_coarse = (ushort)((NumericUpDown)sender).Value;
        }

        private void PMTgainFine_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.DataPMTGainFine, PMT_gain_fine, (int)((NumericUpDown)sender).Value);
            PMT_gain_fine = (ushort)((NumericUpDown)sender).Value;
        }

        private void PMTgainCoarse_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.DataPMTGainCoarse, PMT_gain_coarse, (int)((NumericUpDown)sender).Value);
            PMT_gain_coarse = (ushort)((NumericUpDown)sender).Value;
        }

        private void PMToffset_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.DataPMTOffset, PMT_offset, (int)((NumericUpDown)sender).Value);
            PMT_offset = (ushort)((NumericUpDown)sender).Value;
        }

        private void LEDPMTgainFine_ValueChanged(object sender, EventArgs e)
        {
            SetResistorValue((int)ChipSelectPins.LEDPMTGain, LEDPMT_gain_fine, (int)((NumericUpDown)sender).Value);
            LEDPMT_gain_fine = (ushort)((NumericUpDown)sender).Value;
        }

        private void cbLog_CheckedChanged(object sender, EventArgs e)
        {
            logging = !logging;
        }

        private void nudSampleRate_ValueChanged(object sender, EventArgs e)
        {
            DesiredRate     = (int)nudSampleRate.Value;
            settingsChanged = true;
            DisableEvents();
            SetupMemoryAllocation();
            EnableEvents();
        }

        private void nudSampleAmount_ValueChanged(object sender, EventArgs e)
        {
            NumSamplesForAllChannels = (int)nudSampleAmount.Value * 8;
            settingsChanged = true;
        }

        private void btnSave_Click(object sender, EventArgs e)
        {
            ULEV02.Properties.Settings.Default.PowerCoarseSetting = (int)LEDpowerCoarse.Value;
            ULEV02.Properties.Settings.Default.PowerFineSetting   = (int)LEDpowerFine.Value;
            ULEV02.Properties.Settings.Default.PulseCoarseSetting = (int)PulseDurationCoarse.Value;
            ULEV02.Properties.Settings.Default.PulseFineSetting   = (int)PulseDurationFine.Value;
            ULEV02.Properties.Settings.Default.GainCoarseSetting  = (int)PMTgainCoarse.Value;
            ULEV02.Properties.Settings.Default.GainFineSetting    = (int)PMTgainFine.Value;
            ULEV02.Properties.Settings.Default.OffsetSetting      = (int)PMToffset.Value;
            ULEV02.Properties.Settings.Default.GainLED            = (int)LEDgain.Value;
            ULEV02.Properties.Settings.Default.RateSetting        = (int)nudSampleRate.Value;
            ULEV02.Properties.Settings.Default.TotalSetting       = (int)nudSampleAmount.Value;
            ULEV02.Properties.Settings.Default.CutoffSetting      = (int)nudCutoff.Value;
            ULEV02.Properties.Settings.Default.Save();
        }

        private void btnLoad_Click(object sender, EventArgs e)
        {
            //ResetResistors();

            //Zero all labels
            LED_Brightness_label.Text  = "0";
            LED_Current_label.Text     = "0";
            LED_Temperature_label.Text = "0";
            LED_Voltage_label.Text     = "0";
            PMT_Flu_label.Text         = "0";
            PMT_gain_label.Text        = "0";
            PMT_offset_label.Text      = "0";

            LEDpowerCoarse.Value      = ULEV02.Properties.Settings.Default.PowerCoarseSetting;
            LEDpowerFine.Value        = ULEV02.Properties.Settings.Default.PowerFineSetting;
            PulseDurationCoarse.Value = ULEV02.Properties.Settings.Default.PulseCoarseSetting;
            PulseDurationFine.Value   = ULEV02.Properties.Settings.Default.PulseFineSetting;
            PMTgainCoarse.Value       = ULEV02.Properties.Settings.Default.GainCoarseSetting;
            PMTgainFine.Value         = ULEV02.Properties.Settings.Default.GainFineSetting;
            PMToffset.Value           = ULEV02.Properties.Settings.Default.OffsetSetting;
            nudSampleRate.Value       = ULEV02.Properties.Settings.Default.RateSetting;
            nudSampleAmount.Value     = ULEV02.Properties.Settings.Default.TotalSetting;
            nudCutoff.Value           = ULEV02.Properties.Settings.Default.CutoffSetting;
        }

        private void nudCutoff_ValueChanged(object sender, EventArgs e)
        {
            Cutoff = (float)nudCutoff.Value / 100;
            settingsChanged = true;
        }

        private void btnStart_Click(object sender, EventArgs e)
        {
            measurementTimer.Start();
            cmdStart.Enabled    = false;
            btnStart.Enabled    = false;
            nudInterval.Enabled = false;
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            measurementTimer.Stop();
            cmdStart.Enabled    = true;
            btnStart.Enabled    = true;
            nudInterval.Enabled = true;
        }

        private void nudInterval_ValueChanged(object sender, EventArgs e)
        {
            InitTimer();
        }

        private void cbAutotune_CheckedChanged(object sender, EventArgs e)
        {
            autotune = !autotune;
            if (autotune)
            {
                LEDpowerFine.Value   = 50;
                LEDpowerCoarse.Value = 50;
                settingsChanged      = true;
            }

            gbAutotune.Enabled = autotune;
        }

        private void nudDesiredBrightness_ValueChanged(object sender, EventArgs e)
        {
            desiredBrightness = (float)nudDesiredBrightness.Value;
        }

        private void nudBrightnessTolerance_ValueChanged(object sender, EventArgs e)
        {
            brightnessTolerance = (float)nudBrightnessTolerance.Value;
        }

        private void nudCoarseStep_ValueChanged(object sender, EventArgs e)
        {
            coarseStep = (float)nudCoarseStep.Value;
        }

        private void nudFineStep_ValueChanged(object sender, EventArgs e)
        {
            fineStep = (float)nudFineStep.Value;
        }

        #endregion

        #region Windows Form Designer generated code
        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            this.panel1 = new System.Windows.Forms.Panel();
            this.cbAutotune = new System.Windows.Forms.CheckBox();
            this.panel3 = new System.Windows.Forms.Panel();
            this.LEDgain = new System.Windows.Forms.NumericUpDown();
            this.label22 = new System.Windows.Forms.Label();
            this.label0 = new System.Windows.Forms.Label();
            this.PMToffset = new System.Windows.Forms.NumericUpDown();
            this.label7 = new System.Windows.Forms.Label();
            this.PMTgainCoarse = new System.Windows.Forms.NumericUpDown();
            this.LEDpowerFine = new System.Windows.Forms.NumericUpDown();
            this.PulseDurationCoarse = new System.Windows.Forms.NumericUpDown();
            this.PulseDurationFine = new System.Windows.Forms.NumericUpDown();
            this.nudInterval = new System.Windows.Forms.NumericUpDown();
            this.label6 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.PMTgainFine = new System.Windows.Forms.NumericUpDown();
            this.LEDpowerCoarse = new System.Windows.Forms.NumericUpDown();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.nudSampleRate = new System.Windows.Forms.NumericUpDown();
            this.btnSave = new System.Windows.Forms.Button();
            this.label15 = new System.Windows.Forms.Label();
            this.btnLoad = new System.Windows.Forms.Button();
            this.nudSampleAmount = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.btnStop = new System.Windows.Forms.Button();
            this.btnStart = new System.Windows.Forms.Button();
            this.graph = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.panel2 = new System.Windows.Forms.Panel();
            this.cbFlu = new System.Windows.Forms.CheckBox();
            this.cbOffset = new System.Windows.Forms.CheckBox();
            this.cbGain = new System.Windows.Forms.CheckBox();
            this.cbTemperature = new System.Windows.Forms.CheckBox();
            this.cbBrightness = new System.Windows.Forms.CheckBox();
            this.cbCurrent = new System.Windows.Forms.CheckBox();
            this.cbVoltage = new System.Windows.Forms.CheckBox();
            this.label8 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.nudCutoff = new System.Windows.Forms.NumericUpDown();
            this.label13 = new System.Windows.Forms.Label();
            this.cbLog = new System.Windows.Forms.CheckBox();
            this.label16 = new System.Windows.Forms.Label();
            this.LED_Current_label = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.LED_Voltage_label = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.LED_Brightness_label = new System.Windows.Forms.Label();
            this.LED_Temperature_label = new System.Windows.Forms.Label();
            this.PMT_Flu_label = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.PMT_offset_label = new System.Windows.Forms.Label();
            this.PMT_gain_label = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.cmdStart = new System.Windows.Forms.Button();
            this.gbAutotune = new System.Windows.Forms.GroupBox();
            this.label20 = new System.Windows.Forms.Label();
            this.nudBrightnessTolerance = new System.Windows.Forms.NumericUpDown();
            this.nudDesiredBrightness = new System.Windows.Forms.NumericUpDown();
            this.label19 = new System.Windows.Forms.Label();
            this.nudFineStep = new System.Windows.Forms.NumericUpDown();
            this.nudCoarseStep = new System.Windows.Forms.NumericUpDown();
            this.label17 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.awd = new System.Windows.Forms.Label();
            this.panel1.SuspendLayout();
            this.panel3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LEDgain)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PMToffset)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PMTgainCoarse)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.LEDpowerFine)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PulseDurationCoarse)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PulseDurationFine)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudInterval)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PMTgainFine)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.LEDpowerCoarse)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSampleRate)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSampleAmount)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.graph)).BeginInit();
            this.panel2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudCutoff)).BeginInit();
            this.gbAutotune.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudBrightnessTolerance)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudDesiredBrightness)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudFineStep)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudCoarseStep)).BeginInit();
            this.SuspendLayout();
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.Cornsilk;
            this.panel1.Controls.Add(this.cbAutotune);
            this.panel1.Controls.Add(this.panel3);
            this.panel1.Controls.Add(this.btnStop);
            this.panel1.Controls.Add(this.btnStart);
            this.panel1.Controls.Add(this.graph);
            this.panel1.Controls.Add(this.panel2);
            this.panel1.Controls.Add(this.cmdStart);
            this.panel1.Controls.Add(this.gbAutotune);
            this.panel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel1.Location = new System.Drawing.Point(0, 0);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(723, 664);
            this.panel1.TabIndex = 0;
            // 
            // cbAutotune
            // 
            this.cbAutotune.AutoSize = true;
            this.cbAutotune.Font = new System.Drawing.Font("Segoe UI", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cbAutotune.Location = new System.Drawing.Point(287, 10);
            this.cbAutotune.Name = "cbAutotune";
            this.cbAutotune.Size = new System.Drawing.Size(89, 17);
            this.cbAutotune.TabIndex = 130;
            this.cbAutotune.Text = "AutoTune ©";
            this.cbAutotune.UseVisualStyleBackColor = true;
            this.cbAutotune.CheckedChanged += new System.EventHandler(this.cbAutotune_CheckedChanged);
            // 
            // panel3
            // 
            this.panel3.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.panel3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.panel3.Controls.Add(this.LEDgain);
            this.panel3.Controls.Add(this.label22);
            this.panel3.Controls.Add(this.label0);
            this.panel3.Controls.Add(this.PMToffset);
            this.panel3.Controls.Add(this.label7);
            this.panel3.Controls.Add(this.PMTgainCoarse);
            this.panel3.Controls.Add(this.LEDpowerFine);
            this.panel3.Controls.Add(this.PulseDurationCoarse);
            this.panel3.Controls.Add(this.PulseDurationFine);
            this.panel3.Controls.Add(this.nudInterval);
            this.panel3.Controls.Add(this.label6);
            this.panel3.Controls.Add(this.label1);
            this.panel3.Controls.Add(this.PMTgainFine);
            this.panel3.Controls.Add(this.LEDpowerCoarse);
            this.panel3.Controls.Add(this.label9);
            this.panel3.Controls.Add(this.label10);
            this.panel3.Controls.Add(this.label5);
            this.panel3.Controls.Add(this.nudSampleRate);
            this.panel3.Controls.Add(this.btnSave);
            this.panel3.Controls.Add(this.label15);
            this.panel3.Controls.Add(this.btnLoad);
            this.panel3.Controls.Add(this.nudSampleAmount);
            this.panel3.Controls.Add(this.label2);
            this.panel3.Location = new System.Drawing.Point(0, 0);
            this.panel3.Name = "panel3";
            this.panel3.Size = new System.Drawing.Size(274, 325);
            this.panel3.TabIndex = 136;
            // 
            // LEDgain
            // 
            this.LEDgain.BackColor = System.Drawing.SystemColors.HighlightText;
            this.LEDgain.Location = new System.Drawing.Point(138, 134);
            this.LEDgain.Name = "LEDgain";
            this.LEDgain.Size = new System.Drawing.Size(61, 20);
            this.LEDgain.TabIndex = 134;
            this.LEDgain.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.LEDgain.ValueChanged += new System.EventHandler(this.LEDPMTgainFine_ValueChanged);
            // 
            // label22
            // 
            this.label22.AutoSize = true;
            this.label22.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label22.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label22.Location = new System.Drawing.Point(30, 137);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(102, 17);
            this.label22.TabIndex = 130;
            this.label22.Text = "LED PMT Gain";
            // 
            // label0
            // 
            this.label0.AutoSize = true;
            this.label0.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label0.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label0.Location = new System.Drawing.Point(54, 33);
            this.label0.Name = "label0";
            this.label0.Size = new System.Drawing.Size(78, 17);
            this.label0.TabIndex = 77;
            this.label0.Text = "LED Power";
            // 
            // PMToffset
            // 
            this.PMToffset.BackColor = System.Drawing.SystemColors.HighlightText;
            this.PMToffset.Location = new System.Drawing.Point(138, 108);
            this.PMToffset.Name = "PMToffset";
            this.PMToffset.Size = new System.Drawing.Size(61, 20);
            this.PMToffset.TabIndex = 84;
            this.PMToffset.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.PMToffset.ValueChanged += new System.EventHandler(this.PMToffset_ValueChanged);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label7.Location = new System.Drawing.Point(53, 111);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(79, 17);
            this.label7.TabIndex = 83;
            this.label7.Text = "PMT Offset";
            // 
            // PMTgainCoarse
            // 
            this.PMTgainCoarse.BackColor = System.Drawing.SystemColors.HighlightText;
            this.PMTgainCoarse.Location = new System.Drawing.Point(138, 82);
            this.PMTgainCoarse.Name = "PMTgainCoarse";
            this.PMTgainCoarse.Size = new System.Drawing.Size(61, 20);
            this.PMTgainCoarse.TabIndex = 82;
            this.PMTgainCoarse.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.PMTgainCoarse.ValueChanged += new System.EventHandler(this.PMTgainCoarse_ValueChanged);
            // 
            // LEDpowerFine
            // 
            this.LEDpowerFine.BackColor = System.Drawing.SystemColors.HighlightText;
            this.LEDpowerFine.Location = new System.Drawing.Point(205, 30);
            this.LEDpowerFine.Name = "LEDpowerFine";
            this.LEDpowerFine.Size = new System.Drawing.Size(61, 20);
            this.LEDpowerFine.TabIndex = 95;
            this.LEDpowerFine.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.LEDpowerFine.ValueChanged += new System.EventHandler(this.LEDpowerFine_ValueChanged);
            // 
            // PulseDurationCoarse
            // 
            this.PulseDurationCoarse.BackColor = System.Drawing.SystemColors.HighlightText;
            this.PulseDurationCoarse.Location = new System.Drawing.Point(138, 56);
            this.PulseDurationCoarse.Name = "PulseDurationCoarse";
            this.PulseDurationCoarse.Size = new System.Drawing.Size(61, 20);
            this.PulseDurationCoarse.TabIndex = 80;
            this.PulseDurationCoarse.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.PulseDurationCoarse.ValueChanged += new System.EventHandler(this.PulseDurationCoarse_ValueChanged);
            // 
            // PulseDurationFine
            // 
            this.PulseDurationFine.BackColor = System.Drawing.SystemColors.HighlightText;
            this.PulseDurationFine.Location = new System.Drawing.Point(205, 56);
            this.PulseDurationFine.Name = "PulseDurationFine";
            this.PulseDurationFine.Size = new System.Drawing.Size(61, 20);
            this.PulseDurationFine.TabIndex = 96;
            this.PulseDurationFine.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.PulseDurationFine.ValueChanged += new System.EventHandler(this.PulseDurationFine_ValueChanged);
            // 
            // nudInterval
            // 
            this.nudInterval.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudInterval.Location = new System.Drawing.Point(140, 258);
            this.nudInterval.Maximum = new decimal(new int[] {
            300,
            0,
            0,
            0});
            this.nudInterval.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudInterval.Name = "nudInterval";
            this.nudInterval.Size = new System.Drawing.Size(61, 20);
            this.nudInterval.TabIndex = 129;
            this.nudInterval.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudInterval.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudInterval.ValueChanged += new System.EventHandler(this.nudInterval_ValueChanged);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label6.Location = new System.Drawing.Point(31, 59);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(101, 17);
            this.label6.TabIndex = 79;
            this.label6.Text = "Pulse Duration";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label1.Location = new System.Drawing.Point(0, 261);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(134, 17);
            this.label1.TabIndex = 128;
            this.label1.Text = "Measure Interval (s)";
            // 
            // PMTgainFine
            // 
            this.PMTgainFine.BackColor = System.Drawing.SystemColors.HighlightText;
            this.PMTgainFine.Location = new System.Drawing.Point(205, 82);
            this.PMTgainFine.Name = "PMTgainFine";
            this.PMTgainFine.Size = new System.Drawing.Size(61, 20);
            this.PMTgainFine.TabIndex = 97;
            this.PMTgainFine.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.PMTgainFine.ValueChanged += new System.EventHandler(this.PMTgainFine_ValueChanged);
            // 
            // LEDpowerCoarse
            // 
            this.LEDpowerCoarse.BackColor = System.Drawing.SystemColors.HighlightText;
            this.LEDpowerCoarse.Location = new System.Drawing.Point(138, 30);
            this.LEDpowerCoarse.Name = "LEDpowerCoarse";
            this.LEDpowerCoarse.Size = new System.Drawing.Size(61, 20);
            this.LEDpowerCoarse.TabIndex = 78;
            this.LEDpowerCoarse.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.LEDpowerCoarse.ValueChanged += new System.EventHandler(this.LEDpowerCoarse_ValueChanged);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.ForeColor = System.Drawing.Color.Crimson;
            this.label9.Location = new System.Drawing.Point(142, 4);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(59, 17);
            this.label9.TabIndex = 98;
            this.label9.Text = "Coarse";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.ForeColor = System.Drawing.Color.Crimson;
            this.label10.Location = new System.Drawing.Point(218, 4);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(39, 17);
            this.label10.TabIndex = 99;
            this.label10.Text = "Fine";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label5.Location = new System.Drawing.Point(61, 85);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(71, 17);
            this.label5.TabIndex = 81;
            this.label5.Text = "PMT Gain";
            // 
            // nudSampleRate
            // 
            this.nudSampleRate.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudSampleRate.Enabled = false;
            this.nudSampleRate.Increment = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            this.nudSampleRate.Location = new System.Drawing.Point(140, 206);
            this.nudSampleRate.Maximum = new decimal(new int[] {
            6250,
            0,
            0,
            0});
            this.nudSampleRate.Name = "nudSampleRate";
            this.nudSampleRate.ReadOnly = true;
            this.nudSampleRate.Size = new System.Drawing.Size(61, 20);
            this.nudSampleRate.TabIndex = 119;
            this.nudSampleRate.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudSampleRate.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            this.nudSampleRate.ValueChanged += new System.EventHandler(this.nudSampleRate_ValueChanged);
            // 
            // btnSave
            // 
            this.btnSave.BackColor = System.Drawing.Color.Linen;
            this.btnSave.Location = new System.Drawing.Point(53, 292);
            this.btnSave.Name = "btnSave";
            this.btnSave.Size = new System.Drawing.Size(79, 23);
            this.btnSave.TabIndex = 108;
            this.btnSave.Text = "Save";
            this.btnSave.UseVisualStyleBackColor = false;
            this.btnSave.Click += new System.EventHandler(this.btnSave_Click);
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label15.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label15.Location = new System.Drawing.Point(45, 206);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(89, 17);
            this.label15.TabIndex = 118;
            this.label15.Text = "Sample Rate";
            // 
            // btnLoad
            // 
            this.btnLoad.BackColor = System.Drawing.Color.Linen;
            this.btnLoad.Location = new System.Drawing.Point(138, 292);
            this.btnLoad.Name = "btnLoad";
            this.btnLoad.Size = new System.Drawing.Size(79, 23);
            this.btnLoad.TabIndex = 111;
            this.btnLoad.Text = "Load";
            this.btnLoad.UseVisualStyleBackColor = false;
            this.btnLoad.Click += new System.EventHandler(this.btnLoad_Click);
            // 
            // nudSampleAmount
            // 
            this.nudSampleAmount.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudSampleAmount.Location = new System.Drawing.Point(140, 232);
            this.nudSampleAmount.Maximum = new decimal(new int[] {
            6250,
            0,
            0,
            0});
            this.nudSampleAmount.Name = "nudSampleAmount";
            this.nudSampleAmount.Size = new System.Drawing.Size(61, 20);
            this.nudSampleAmount.TabIndex = 117;
            this.nudSampleAmount.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudSampleAmount.Value = new decimal(new int[] {
            30,
            0,
            0,
            0});
            this.nudSampleAmount.ValueChanged += new System.EventHandler(this.nudSampleAmount_ValueChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.SystemColors.ControlText;
            this.label2.Location = new System.Drawing.Point(44, 235);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(90, 17);
            this.label2.TabIndex = 114;
            this.label2.Text = "# of Samples";
            // 
            // btnStop
            // 
            this.btnStop.BackColor = System.Drawing.Color.Red;
            this.btnStop.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.btnStop.Location = new System.Drawing.Point(364, 298);
            this.btnStop.Name = "btnStop";
            this.btnStop.Size = new System.Drawing.Size(79, 23);
            this.btnStop.TabIndex = 127;
            this.btnStop.Text = "Stop";
            this.btnStop.UseVisualStyleBackColor = false;
            this.btnStop.Click += new System.EventHandler(this.btnStop_Click);
            // 
            // btnStart
            // 
            this.btnStart.BackColor = System.Drawing.Color.Green;
            this.btnStart.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.btnStart.Location = new System.Drawing.Point(280, 298);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(79, 23);
            this.btnStart.TabIndex = 126;
            this.btnStart.Text = "Start";
            this.btnStart.UseVisualStyleBackColor = false;
            this.btnStart.Click += new System.EventHandler(this.btnStart_Click);
            // 
            // graph
            // 
            this.graph.BackColor = System.Drawing.Color.Cornsilk;
            chartArea1.Name = "ChartArea1";
            this.graph.ChartAreas.Add(chartArea1);
            legend1.Alignment = System.Drawing.StringAlignment.Center;
            legend1.BackColor = System.Drawing.Color.Cornsilk;
            legend1.Docking = System.Windows.Forms.DataVisualization.Charting.Docking.Bottom;
            legend1.Name = "Legend1";
            this.graph.Legends.Add(legend1);
            this.graph.Location = new System.Drawing.Point(2, 331);
            this.graph.Name = "graph";
            series1.ChartArea = "ChartArea1";
            series1.Legend = "Legend1";
            series1.Name = "Series1";
            this.graph.Series.Add(series1);
            this.graph.Size = new System.Drawing.Size(713, 333);
            this.graph.TabIndex = 125;
            this.graph.Text = "chart1";
            // 
            // panel2
            // 
            this.panel2.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.panel2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.panel2.Controls.Add(this.cbFlu);
            this.panel2.Controls.Add(this.cbOffset);
            this.panel2.Controls.Add(this.cbGain);
            this.panel2.Controls.Add(this.cbTemperature);
            this.panel2.Controls.Add(this.cbBrightness);
            this.panel2.Controls.Add(this.cbCurrent);
            this.panel2.Controls.Add(this.cbVoltage);
            this.panel2.Controls.Add(this.label8);
            this.panel2.Controls.Add(this.label14);
            this.panel2.Controls.Add(this.nudCutoff);
            this.panel2.Controls.Add(this.label13);
            this.panel2.Controls.Add(this.cbLog);
            this.panel2.Controls.Add(this.label16);
            this.panel2.Controls.Add(this.LED_Current_label);
            this.panel2.Controls.Add(this.label12);
            this.panel2.Controls.Add(this.LED_Voltage_label);
            this.panel2.Controls.Add(this.label11);
            this.panel2.Controls.Add(this.LED_Brightness_label);
            this.panel2.Controls.Add(this.LED_Temperature_label);
            this.panel2.Controls.Add(this.PMT_Flu_label);
            this.panel2.Controls.Add(this.label3);
            this.panel2.Controls.Add(this.PMT_offset_label);
            this.panel2.Controls.Add(this.PMT_gain_label);
            this.panel2.Controls.Add(this.label4);
            this.panel2.Location = new System.Drawing.Point(447, 0);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(276, 215);
            this.panel2.TabIndex = 124;
            // 
            // cbFlu
            // 
            this.cbFlu.AutoSize = true;
            this.cbFlu.Location = new System.Drawing.Point(4, 132);
            this.cbFlu.Name = "cbFlu";
            this.cbFlu.Size = new System.Drawing.Size(15, 14);
            this.cbFlu.TabIndex = 129;
            this.cbFlu.UseVisualStyleBackColor = true;
            // 
            // cbOffset
            // 
            this.cbOffset.AutoSize = true;
            this.cbOffset.Location = new System.Drawing.Point(4, 111);
            this.cbOffset.Name = "cbOffset";
            this.cbOffset.Size = new System.Drawing.Size(15, 14);
            this.cbOffset.TabIndex = 128;
            this.cbOffset.UseVisualStyleBackColor = true;
            // 
            // cbGain
            // 
            this.cbGain.AutoSize = true;
            this.cbGain.Location = new System.Drawing.Point(4, 90);
            this.cbGain.Name = "cbGain";
            this.cbGain.Size = new System.Drawing.Size(15, 14);
            this.cbGain.TabIndex = 127;
            this.cbGain.UseVisualStyleBackColor = true;
            // 
            // cbTemperature
            // 
            this.cbTemperature.AutoSize = true;
            this.cbTemperature.Location = new System.Drawing.Point(4, 69);
            this.cbTemperature.Name = "cbTemperature";
            this.cbTemperature.Size = new System.Drawing.Size(15, 14);
            this.cbTemperature.TabIndex = 126;
            this.cbTemperature.UseVisualStyleBackColor = true;
            // 
            // cbBrightness
            // 
            this.cbBrightness.AutoSize = true;
            this.cbBrightness.Location = new System.Drawing.Point(4, 48);
            this.cbBrightness.Name = "cbBrightness";
            this.cbBrightness.Size = new System.Drawing.Size(15, 14);
            this.cbBrightness.TabIndex = 125;
            this.cbBrightness.UseVisualStyleBackColor = true;
            // 
            // cbCurrent
            // 
            this.cbCurrent.AutoSize = true;
            this.cbCurrent.Location = new System.Drawing.Point(4, 6);
            this.cbCurrent.Name = "cbCurrent";
            this.cbCurrent.Size = new System.Drawing.Size(15, 14);
            this.cbCurrent.TabIndex = 124;
            this.cbCurrent.UseVisualStyleBackColor = true;
            // 
            // cbVoltage
            // 
            this.cbVoltage.AutoSize = true;
            this.cbVoltage.Location = new System.Drawing.Point(4, 27);
            this.cbVoltage.Name = "cbVoltage";
            this.cbVoltage.Size = new System.Drawing.Size(15, 14);
            this.cbVoltage.TabIndex = 123;
            this.cbVoltage.UseVisualStyleBackColor = true;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.ForeColor = System.Drawing.Color.Crimson;
            this.label8.Location = new System.Drawing.Point(73, 5);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(86, 17);
            this.label8.TabIndex = 85;
            this.label8.Text = "LED Current";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label14.ForeColor = System.Drawing.Color.Crimson;
            this.label14.Location = new System.Drawing.Point(33, 131);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(126, 17);
            this.label14.TabIndex = 89;
            this.label14.Text = "PMT Fluorescence";
            // 
            // nudCutoff
            // 
            this.nudCutoff.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudCutoff.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.nudCutoff.Location = new System.Drawing.Point(188, 151);
            this.nudCutoff.Name = "nudCutoff";
            this.nudCutoff.Size = new System.Drawing.Size(46, 20);
            this.nudCutoff.TabIndex = 122;
            this.nudCutoff.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudCutoff.Value = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.nudCutoff.ValueChanged += new System.EventHandler(this.nudCutoff_ValueChanged);
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label13.ForeColor = System.Drawing.Color.Crimson;
            this.label13.Location = new System.Drawing.Point(38, 68);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(121, 17);
            this.label13.TabIndex = 88;
            this.label13.Text = "LED Temperature";
            // 
            // cbLog
            // 
            this.cbLog.AutoSize = true;
            this.cbLog.Location = new System.Drawing.Point(91, 183);
            this.cbLog.Name = "cbLog";
            this.cbLog.Size = new System.Drawing.Size(82, 17);
            this.cbLog.TabIndex = 110;
            this.cbLog.Text = "Log Results";
            this.cbLog.UseVisualStyleBackColor = true;
            this.cbLog.CheckedChanged += new System.EventHandler(this.cbLog_CheckedChanged);
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label16.ForeColor = System.Drawing.Color.Crimson;
            this.label16.Location = new System.Drawing.Point(47, 151);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(112, 17);
            this.label16.TabIndex = 121;
            this.label16.Text = "Sample Cutoff %";
            // 
            // LED_Current_label
            // 
            this.LED_Current_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LED_Current_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.LED_Current_label.Location = new System.Drawing.Point(156, 5);
            this.LED_Current_label.Name = "LED_Current_label";
            this.LED_Current_label.Size = new System.Drawing.Size(110, 15);
            this.LED_Current_label.TabIndex = 90;
            this.LED_Current_label.Text = "0";
            this.LED_Current_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label12.ForeColor = System.Drawing.Color.Crimson;
            this.label12.Location = new System.Drawing.Point(53, 47);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(106, 17);
            this.label12.TabIndex = 87;
            this.label12.Text = "LED Brightness";
            // 
            // LED_Voltage_label
            // 
            this.LED_Voltage_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LED_Voltage_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.LED_Voltage_label.Location = new System.Drawing.Point(156, 26);
            this.LED_Voltage_label.Name = "LED_Voltage_label";
            this.LED_Voltage_label.Size = new System.Drawing.Size(110, 15);
            this.LED_Voltage_label.TabIndex = 91;
            this.LED_Voltage_label.Text = "0";
            this.LED_Voltage_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label11.ForeColor = System.Drawing.Color.Crimson;
            this.label11.Location = new System.Drawing.Point(72, 26);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(87, 17);
            this.label11.TabIndex = 86;
            this.label11.Text = "LED Voltage";
            // 
            // LED_Brightness_label
            // 
            this.LED_Brightness_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LED_Brightness_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.LED_Brightness_label.Location = new System.Drawing.Point(156, 47);
            this.LED_Brightness_label.Name = "LED_Brightness_label";
            this.LED_Brightness_label.Size = new System.Drawing.Size(110, 15);
            this.LED_Brightness_label.TabIndex = 92;
            this.LED_Brightness_label.Text = "0";
            this.LED_Brightness_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // LED_Temperature_label
            // 
            this.LED_Temperature_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LED_Temperature_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.LED_Temperature_label.Location = new System.Drawing.Point(156, 68);
            this.LED_Temperature_label.Name = "LED_Temperature_label";
            this.LED_Temperature_label.Size = new System.Drawing.Size(110, 15);
            this.LED_Temperature_label.TabIndex = 93;
            this.LED_Temperature_label.Text = "0";
            this.LED_Temperature_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // PMT_Flu_label
            // 
            this.PMT_Flu_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.PMT_Flu_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.PMT_Flu_label.Location = new System.Drawing.Point(156, 131);
            this.PMT_Flu_label.Name = "PMT_Flu_label";
            this.PMT_Flu_label.Size = new System.Drawing.Size(110, 15);
            this.PMT_Flu_label.TabIndex = 94;
            this.PMT_Flu_label.Text = "0";
            this.PMT_Flu_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.Color.Crimson;
            this.label3.Location = new System.Drawing.Point(88, 89);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(71, 17);
            this.label3.TabIndex = 103;
            this.label3.Text = "PMT Gain";
            // 
            // PMT_offset_label
            // 
            this.PMT_offset_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.PMT_offset_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.PMT_offset_label.Location = new System.Drawing.Point(156, 110);
            this.PMT_offset_label.Name = "PMT_offset_label";
            this.PMT_offset_label.Size = new System.Drawing.Size(110, 15);
            this.PMT_offset_label.TabIndex = 107;
            this.PMT_offset_label.Text = "0";
            this.PMT_offset_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // PMT_gain_label
            // 
            this.PMT_gain_label.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.PMT_gain_label.ForeColor = System.Drawing.SystemColors.ControlText;
            this.PMT_gain_label.Location = new System.Drawing.Point(156, 89);
            this.PMT_gain_label.Name = "PMT_gain_label";
            this.PMT_gain_label.Size = new System.Drawing.Size(110, 15);
            this.PMT_gain_label.TabIndex = 104;
            this.PMT_gain_label.Text = "0";
            this.PMT_gain_label.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ForeColor = System.Drawing.Color.Crimson;
            this.label4.Location = new System.Drawing.Point(80, 110);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(79, 17);
            this.label4.TabIndex = 106;
            this.label4.Text = "PMT Offset";
            // 
            // cmdStart
            // 
            this.cmdStart.BackColor = System.Drawing.Color.SeaGreen;
            this.cmdStart.Cursor = System.Windows.Forms.Cursors.Default;
            this.cmdStart.Font = new System.Drawing.Font("Arial", 8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cmdStart.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.cmdStart.Location = new System.Drawing.Point(280, 259);
            this.cmdStart.Name = "cmdStart";
            this.cmdStart.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.cmdStart.Size = new System.Drawing.Size(161, 33);
            this.cmdStart.TabIndex = 73;
            this.cmdStart.Text = "Measure";
            this.cmdStart.UseVisualStyleBackColor = false;
            this.cmdStart.Click += new System.EventHandler(this.cmdStart_Click);
            // 
            // gbAutotune
            // 
            this.gbAutotune.BackColor = System.Drawing.Color.Cornsilk;
            this.gbAutotune.Controls.Add(this.label20);
            this.gbAutotune.Controls.Add(this.nudBrightnessTolerance);
            this.gbAutotune.Controls.Add(this.nudDesiredBrightness);
            this.gbAutotune.Controls.Add(this.label19);
            this.gbAutotune.Controls.Add(this.nudFineStep);
            this.gbAutotune.Controls.Add(this.nudCoarseStep);
            this.gbAutotune.Controls.Add(this.label17);
            this.gbAutotune.Controls.Add(this.label18);
            this.gbAutotune.Enabled = false;
            this.gbAutotune.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.gbAutotune.Location = new System.Drawing.Point(279, 10);
            this.gbAutotune.Name = "gbAutotune";
            this.gbAutotune.Size = new System.Drawing.Size(161, 166);
            this.gbAutotune.TabIndex = 139;
            this.gbAutotune.TabStop = false;
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.Location = new System.Drawing.Point(14, 85);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(55, 13);
            this.label20.TabIndex = 141;
            this.label20.Text = "Tolerance";
            // 
            // nudBrightnessTolerance
            // 
            this.nudBrightnessTolerance.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudBrightnessTolerance.DecimalPlaces = 2;
            this.nudBrightnessTolerance.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.nudBrightnessTolerance.Location = new System.Drawing.Point(85, 82);
            this.nudBrightnessTolerance.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudBrightnessTolerance.Name = "nudBrightnessTolerance";
            this.nudBrightnessTolerance.Size = new System.Drawing.Size(61, 20);
            this.nudBrightnessTolerance.TabIndex = 140;
            this.nudBrightnessTolerance.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudBrightnessTolerance.Value = new decimal(new int[] {
            2,
            0,
            0,
            131072});
            this.nudBrightnessTolerance.ValueChanged += new System.EventHandler(this.nudBrightnessTolerance_ValueChanged);
            // 
            // nudDesiredBrightness
            // 
            this.nudDesiredBrightness.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudDesiredBrightness.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.nudDesiredBrightness.DecimalPlaces = 2;
            this.nudDesiredBrightness.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.nudDesiredBrightness.Location = new System.Drawing.Point(40, 46);
            this.nudDesiredBrightness.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.nudDesiredBrightness.Name = "nudDesiredBrightness";
            this.nudDesiredBrightness.Size = new System.Drawing.Size(79, 20);
            this.nudDesiredBrightness.TabIndex = 139;
            this.nudDesiredBrightness.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudDesiredBrightness.Value = new decimal(new int[] {
            17,
            0,
            0,
            65536});
            this.nudDesiredBrightness.ValueChanged += new System.EventHandler(this.nudDesiredBrightness_ValueChanged);
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label19.ForeColor = System.Drawing.Color.Crimson;
            this.label19.Location = new System.Drawing.Point(15, 25);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(129, 15);
            this.label19.TabIndex = 138;
            this.label19.Text = "Desired Brightness (V)";
            // 
            // nudFineStep
            // 
            this.nudFineStep.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudFineStep.DecimalPlaces = 2;
            this.nudFineStep.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.nudFineStep.Location = new System.Drawing.Point(85, 135);
            this.nudFineStep.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudFineStep.Name = "nudFineStep";
            this.nudFineStep.Size = new System.Drawing.Size(61, 20);
            this.nudFineStep.TabIndex = 131;
            this.nudFineStep.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudFineStep.Value = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.nudFineStep.ValueChanged += new System.EventHandler(this.nudFineStep_ValueChanged);
            // 
            // nudCoarseStep
            // 
            this.nudCoarseStep.BackColor = System.Drawing.SystemColors.HighlightText;
            this.nudCoarseStep.DecimalPlaces = 2;
            this.nudCoarseStep.Increment = new decimal(new int[] {
            1,
            0,
            0,
            131072});
            this.nudCoarseStep.Location = new System.Drawing.Point(85, 109);
            this.nudCoarseStep.Maximum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudCoarseStep.Name = "nudCoarseStep";
            this.nudCoarseStep.Size = new System.Drawing.Size(61, 20);
            this.nudCoarseStep.TabIndex = 132;
            this.nudCoarseStep.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.nudCoarseStep.Value = new decimal(new int[] {
            18,
            0,
            0,
            131072});
            this.nudCoarseStep.ValueChanged += new System.EventHandler(this.nudCoarseStep_ValueChanged);
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.Location = new System.Drawing.Point(14, 112);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(65, 13);
            this.label17.TabIndex = 133;
            this.label17.Text = "Coarse Step";
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.Location = new System.Drawing.Point(14, 138);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(52, 13);
            this.label18.TabIndex = 134;
            this.label18.Text = "Fine Step";
            // 
            // awd
            // 
            this.awd.AutoSize = true;
            this.awd.Font = new System.Drawing.Font("Consolas", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.awd.ForeColor = System.Drawing.Color.Cyan;
            this.awd.Location = new System.Drawing.Point(256, 81);
            this.awd.Name = "awd";
            this.awd.Size = new System.Drawing.Size(48, 17);
            this.awd.TabIndex = 109;
            this.awd.Text = "~30ms";
            // 
            // frmEventDisplay
            // 
            this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
            this.ClientSize = new System.Drawing.Size(723, 664);
            this.Controls.Add(this.panel1);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.Name = "frmEventDisplay";
            this.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.frmEventDisplay_FormClosing);
            this.Load += new System.EventHandler(this.frmEventDisplay_Load);
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.panel3.ResumeLayout(false);
            this.panel3.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LEDgain)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PMToffset)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PMTgainCoarse)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.LEDpowerFine)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PulseDurationCoarse)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PulseDurationFine)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudInterval)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PMTgainFine)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.LEDpowerCoarse)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSampleRate)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudSampleAmount)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.graph)).EndInit();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudCutoff)).EndInit();
            this.gbAutotune.ResumeLayout(false);
            this.gbAutotune.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudBrightnessTolerance)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudDesiredBrightness)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudFineStep)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudCoarseStep)).EndInit();
            this.ResumeLayout(false);

        }

        private void InitUL()
        {

            ErrorInfo ULStat;

            //  Initiate error handling
            //   activating error handling will trap errors like
            //   bad channel numbers and non-configured conditions.
            //   Parameters:
            //     ErrorReporting.PrintAll :all warnings and errors encountered will be printed
            //     ErrorHandling.StopAll   :if an error is encountered, the program will stop

            clsErrorDefs.ReportError = ErrorReporting.PrintAll;
            clsErrorDefs.HandleError = ErrorHandling.StopAll;
            ULStat = MccService.ErrHandling
                (ErrorReporting.PrintAll, ErrorHandling.StopAll);

        }
        #endregion

        #region Form initialization and entry point


        [STAThread]
        static void Main()
        {
            Application.Run(new frmEventDisplay());
        }

        public frmEventDisplay()
        {
            InitializeComponent();
        }

        #endregion
    }
}