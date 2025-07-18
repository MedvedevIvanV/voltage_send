## Generated SDC file "signal_adc_fsmc.sdc"

## Copyright (C) 2021  Intel Corporation. All rights reserved.
## Your use of Intel Corporation's design tools, logic functions 
## and other software and tools, and any partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Intel Program License 
## Subscription Agreement, the Intel Quartus Prime License Agreement,
## the Intel FPGA IP License Agreement, or other applicable license
## agreement, including, without limitation, that your use is for
## the sole purpose of programming logic devices manufactured by
## Intel and sold by Intel or its authorized distributors.  Please
## refer to the applicable agreement for further details, at
## https://fpgasoftware.intel.com/eula.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 21.1.0 Build 842 10/21/2021 SJ Lite Edition"

## DATE    "Thu May 29 11:04:10 2025"

##
## DEVICE  "10CL006YE144A7G"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

##create_clock -name {clk_20mhz} -period 50.000 -waveform { 0.000 25.000 } [get_ports {clk_20mhz}]

create_clock -name clk_20mhz -period 50.000 [get_ports clk_20mhz]
derive_pll_clocks
derive_clock_uncertainty

create_clock -name {CLK_P} -period 12.500 [get_ports {CLK_P}]
set_clock_groups -asynchronous -group {CLK_P}


#**************************************************************
# Create Generated Clock
#**************************************************************



#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -hold 0.020  
set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -hold 0.020  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -hold 0.020  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -hold 0.020  


#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************



#**************************************************************
# Set False Path
#**************************************************************




#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

