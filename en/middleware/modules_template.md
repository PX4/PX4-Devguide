# Modules Reference: Template

## module
Source: [templates/module](https://github.com/PX4/Firmware/tree/master/src/templates/module)


### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
```
module start -f -p 42
```


### Usage {#module_usage}
```
module <command> [arguments...]
 Commands:
   start
     [-f]        Optional example flag
     [-p <val>]  Optional example parameter
                 default: 0

   stop

   status        print status info
```
