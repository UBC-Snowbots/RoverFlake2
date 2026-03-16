# changed

Other than refactoring:

- MoteusIO: Changed send_position_cmds
    - removed the old parsing hacks because we are using moteus now (which doesn't sent UART text anymore)
    - made it to use actual motor response because previously it used commanded values which just doesn't make sense to me (basically overhead imo)
    - removed initial testing but can add back idk

- changed send_velocity_cmds too
    - now that axis 4 works, i'ma remove the if statement :-)
    