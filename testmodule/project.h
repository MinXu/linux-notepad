/**
 @mainpage
  
 @authors Corporate Technology Group, Harman International. Questions? Contact <a href="mailto:min.xu@harman.com">Min Xu</a>. 
  
 @section intro Introduction 

 This is a test project. We can used the makefile generate a kernel external module ,that invoke a symbol export by another kernel module.
  
 @section parents Parent Modules 

  - None
  
 @section children Sub-modules 
  
  - cmd.ko
  - print.ko
  - printFunc.ko
  - printLine.ko

 @subsection Dependencies

  - kernel module
  
 @section detail1 Brief Description 

 creat a module,invoke a symbol export by another kernel module. insmod via passing paramters
 See the <a href="http://sourcehotel.hmg.ad.harman.com:1667///Sandbox/CTG/project/J4AVB/Design/BEModuleDesign/BEModuleDesign.html">J4 Best Effort Ethernet Module design document</a> for more detials.  
*/