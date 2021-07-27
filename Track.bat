@ECHO OFF
TITLE Starting Satellite Tracking Software
SETLOCAL
::set location to root :C and then remove START /d and see if it just runs

:: GPredict
ECHO Starting Gpredict...
START /d "C:\Users\augus\Documents\DS\Tracking Software" gpredict.lnk
ECHO Gpredict started, starting SDR#

:: SDR#
START /d "C:\Users\augus\Documents\DS\Tracking Software" SDRSharp.lnk
ECHO SDR# started, starting Hamlib...

:: Hamlib
FOR /l %%x in (1,1,7) DO (
	ECHO Trying COM port %%x
	START rotctld -m 202 -s 9600 -r COM%%x
)

PAUSE