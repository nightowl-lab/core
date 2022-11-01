The package is used to configure starneto(Inertial/Satellite Integrated Navigation System) by command.

# Instruction

## System feedback
1. Setup success : $cmd,config,ok* cs<CR><LF>
2. Setup failure : $cmd,Config,failed* cs<CR><LF>
3. Invaid command : $cmd,Bad,Command* cs<CR><LF>

# Command Configure Command

## Save configure command
Command : $cmd,save,config*ff
After saving the configure parameter, the user should restart the device.

## Get the net port configure command
Command : $cmd,get,netpara*ff

## Get the protocol output configure command
Command : $cmd,get,output *ff