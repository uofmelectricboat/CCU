# Boards

Currently defines custom boards to make the ```ACANFD``` library recognize that a ```nucelo_h743zi2``` board is being used, as there is no board with that name defined in ```STSTM32```

```ACANFD```  doesn't technically support the ```nucelo_g431rb```, but it does support the ```nucelo_g474re```. Thus it is redefined here to use the name ```nucelo_g474re``` but the configs of the ```nucelo_g431rb```.
