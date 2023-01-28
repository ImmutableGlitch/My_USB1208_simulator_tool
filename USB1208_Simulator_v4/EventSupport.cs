using System;
using System.Collections.Generic;
using System.Text;
using MccDaq;
using ErrorDefs;

namespace EventSupport
{
    public class clsEventSupport
    {
        public const int DATAEVENT = 1;
        public const int ENDEVENT = 2;
        public const int PRETRIGEVENT = 4;
        public const int ERREVENT = 8;
        public const int ENDOUTEVENT = 16;
        public const int DCHANGEEVENT = 32;
        public const int INTEVENT = 64;

        private MccDaq.MccBoard TestBoard;

        public int FindEventsOfType(MccDaq.MccBoard DaqBoard,
            int EventType)
        {
            MccDaq.ErrorInfo ULStat;

            // check supported features by trial 
            // and error with error handling disabled
            ULStat = MccDaq.MccService.ErrHandling
                (MccDaq.ErrorReporting.DontPrint, MccDaq.ErrorHandling.DontStop);

            TestBoard = DaqBoard;

            // check support of event handling by trial 
            // and error with error handling disabled
            int EventsFound = 0;
            if ((EventType & DCHANGEEVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnChangeOfDigInput);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | DCHANGEEVENT);
            }
            if ((EventType & INTEVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnExternalInterrupt);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | INTEVENT);
            }
            if ((EventType & ERREVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnScanError);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | ERREVENT);
            }
            if ((EventType & DATAEVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnDataAvailable);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | DATAEVENT);
            }
            if ((EventType & ENDEVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnEndOfInputScan);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | ENDEVENT);
            }
            if ((EventType & PRETRIGEVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnPretrigger);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | PRETRIGEVENT);
            }
            if ((EventType & ENDOUTEVENT) > 0)
            {
                ULStat = DaqBoard.DisableEvent(MccDaq.EventType.OnEndOfOutputScan);
                if (ULStat.Value == MccDaq.ErrorInfo.ErrorCode.NoErrors)
                    EventsFound = (EventsFound | ENDOUTEVENT);
            }

            ULStat = MccDaq.MccService.ErrHandling
                (clsErrorDefs.ReportError, clsErrorDefs.HandleError);
            return EventsFound;
        }

    }
}
