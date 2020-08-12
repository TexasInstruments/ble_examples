# Versioning a project based on the BLE-Stack

## Table of Contents

* [Introduction](#introduction)
* [TI Software Overview](#ti-software-overview)
* [Version Control Strategies](#version-control-strategies)
* [What software to version control](#what-software-should-be-version-controlled)
* [Managing Dependencies](#managing-dependencies)
* [Workspace concepts](#workspace-concepts)
* [GitHub as an example](#github-as-an-example)

## Introduction

This guide is intended to serve as a model workflow for users who are
developing application code based on the TI BLE-Stack. The key topics covered
are:

 - TI Software overview and what files should be version controlled
 - Version control strategies
 - Managing dependencies
 - Workspace concepts

Software development workflows are not intended to be one size fits all.
This example is intended to be a foundation, and not a complete development
process. Individual components of the workflow may need to be tweaked to meet
the needs of a product or design team.

Most of these concepts are generic to TI SimpleLink products, and can be reused.
However, this guide seeks to use the `ble_examples` GitHub as a model workflow,
so some concepts are specific to the TI BLE-Stack component of the SimpleLink
SDK.

## TI Software Overview

Before discussing a sample workflow, we will first cover the various
layers provided by the TI SimpleLink SDK, specifically as they pertain to the
BLE-Stack.

**Device Firmware, Kernels**

As detailed in the `simplelink_mcu_sdk\User's_Guide` TI SDKs include all
necessary software for a modern RTOS based system including:

 - An RTOS Kernel
 - Hardware/Register Access Layer (driverlib, device headers, boot code, etc)
 - RTOS Drivers
 - Board files

These software layers are usually provided in source format for reference, but
**are not recommend for modification.** Board files are the exception to this
general rule. These are intended to be changed by the customer and should
be copied into their work-tree.

**Wireless Software Stacks**

These are the protocol stack implementations that implement a given wireless
specification.
In the case of the BLE-Stack component, these are closed source format and
provided in library form. However, there are a number of files that are provided
in source format. The protocol stack files include:

 - Protocol stack libraries: lower portions of the stack provided as statically linked libraries
 - Protocol stack source files: portions of the stack provided in source
 - Protocol stack middleware: ICall or OSAL layers that are provided in source

 These software layers are described in depth by the ble(5)stack-users-guide
 located in `ble(5)stack\ble_user_guide\ble5stack-users-guide`.
 These layers **are not recommended for modification**

**Application Software**

The TI SimpleLink SDK also provides a number of sample applications that often
implement a feature or role of a given protocol stack. These sample applications
are intended as a starting point for customer development. TI intends that the
customer will start developing their user space software based on these
applications. From this starting point, it is intended that the developer will
add features and develop their own application on top of the TI protocol stack.

However, it is generally recommended that the developer
**copies these application files** into their work-tree and starts to modify
them from there. Then it is possible for the developer to refer back to the
original files in the SDK.

## Version Control Strategies

This section seeks to detail a model workflow for version controlling an
application layer code base that is built upon the TI BLE-Stack.
It also aims to describe the various layers of software provided by the TI
SimpleLink SDK and how they play a role in the customer development workflow.


### What software should be version controlled

Based on the summary of the TI provided software above, this section will focus
on what files should be modified by the developer, and how to best do so.

The TI SimpleLink SDK should be treated as a Read Only (RO) dependency that is
required to build the customer application. At the start of a development cycle,
the developer should start with a given TI SimpleLink SDK and based on the
guidance in the protocol stack user guide (i.e. blestack-users-guide) select a
sample application from the SDK as a base for their development.

At this time the customer should copy the **application software** and **board**
files required to a separate work-tree outside of the SimpleLink SDK.
This separate work-tree should include the following:

 * Copies of the sample application files from the base SimpleLink SDK
 * Proprietary application files needed to implement a given use case
 * [optional] copies of protocol stack middleware files if they must be modified

The following diagram shows an illustration of the sample work-tree


```
              SimpleLink SDK/
              ^          /docs
    (links)   |          /examples
              |          /source
              |          /tools
              |          …
              ---Customer work-tree/
              ^          /customer
    (copies)  |          /custom
       or     |          /tree
    (links)   |          /here
              |          …
              |          Project files (.projectspec, .ewp, etc)
              ---Workspace/
                         Files copied to workspace
                         Generated workspace content
```

**SimpleLink SDK**

The SimpleLink SDK should be treated as read only. Customers should copy the
application and board files that they wish to base their development on into a
work-tree outside of the SDK. In this way, the SDK dependency can be updated
and the original content in the SDK can be referenced untouched.

**Customer work-tree**

This is where the application development occurs based on a TI protocol stack
like the BLE-Stack. Files that need to be modified by the customer should
be copied into the separate work-tree, and this folder should be placed
under version control.

**Workspace**

CCS and IAR have the concept of using a workspace as a scratch pad for
developing applications, this generally involves a project recipe file and
copying files from the customer work-tree into a given folder.

**Workspaces include generated files that are not intended to be put under
version control**

Note: BLE-Stack IAR projects do not currently support the workspace concept, the
files will be linked/ directly in the customer work-tree

### Managing Dependencies

In general there are three required software components needed to build, debug,
and deploy software in the SimpleLink ecosystem

 1. IDE/Toolchain (BLE-Stack supports IAR and CCS)
 2. XDC Tools
 3. SimpleLink SDK

The customer application should be built upon the latter two dependencies,
without modification.

If the customer wishes to use an application layer file such as `multi_role.c`
or `simple_gatt_profile.c` from the SDK, it should be copied to their work-tree.

When it is time to upgrade to a newer version of the SimpleLink SDK, the
customer should refer to their base file (i.e. `simple_gatt_profile.c`) and
merge their changes with those from the latest SimpleLink SDK release.

*This is a critical step as there may have been useful bug-fixes or changes in
the files that improve performance* The customer should evaluate these changes
and determine whether or not to incorporate them into their code base (manual 
step).

### Workspace concepts

A user application has dependencies in the SimpleLink SDK and XDCTools.
These should be achieved by a read only link. That is, customer project files
(.ewp, .projectspec, etc) should point directly to the read only SimpleLink SDK
installation. The user should not modify these files. Files that are not
modified by the customer should not be copied into their work-tree or the
workspace.

There is an additional layer of copying that can be enabled when a workspace is
used and certain files are specified to copy into the workspace.

The workspace can be thought of a scratch-pad where changes the customer
application code can be prototyped without changing the contents of the
work-tree. Dependencies from the SimpleLink SDK should not be copied to the
workspace, only files from the customer's work-tree.

Copying to the workspace is optional, and does not need to be used. Instead,
the IDE can link to the the application files in the customer work-tree and link
to the other dependencies in the SimpleLink SDK.

**The workspace is not intended to be version controlled** Additionally, changes
made on files copied to the workspace should be copied back to the work-tree.


## GitHub as an example

This GitHub acts as a working sample of the above workflow.
Each sample application in the GitHub as well as their project files are under
version control. The project files are written so that they link to the SDK
for files that are dependencies and will reference the local working
tree for files that are under version control. In this way, the only code that
needs to be version controlled is the application layer code and TI SimpleLink
code that is modified by the customer, and the project files required to build
the project using an IDE.

 - The files in the GitHub repo will copy certain files to the CCS workspace
 - To disable this change `action="copy"` to `action="link"`

Another strategy is to re-import and build the project every time.  This can be
done via a single command, see here
[CCS Command Line](http://processors.wiki.ti.com/index.php/Projects_-_Command_Line_Build/Create)
