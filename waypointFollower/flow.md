
```flow
st=>start: Start
goal=>condition: Reach the goal?
init=>inputoutput: Load waypoints
getpose=>operation: Get robot current position
getwp=>operation: Get closet waypoints index
stop=>inputoutput: Stop
cmd=>inputoutput: Compute and send velocity cmd 
end=>end: End

st->init
init->goal
goal(no)->getpose(top)
goal(yes,bottom)->stop(top)
getpose->getwp
getwp->cmd
stop->end
cmd->goal
```

