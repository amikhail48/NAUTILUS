<?php
  define('BASE_DIR', dirname(__FILE__));
  require_once(BASE_DIR.'/config.php');
  
  function sys_cmd($cmd) {
    if(strncmp($cmd, "reboot", strlen("reboot")) == 0) {
      shell_exec('sudo shutdown -r now');
    } else if(strncmp($cmd, "shutdown", strlen("shutdown")) == 0) {
      shell_exec('sudo shutdown -h now');
    } else if(strncmp($cmd, "lt_off",strlen("lt_off")) == 0 ) {
      shell_exec('gpio -g write 17 0');
    } else if (strncmp($cmd, "lt_on",strlen("lt_on")) == 0 ) {
      shell_exec('gpio -g write 17 1');
    } else {
      // unknown
    }
  }


  if(isset($_GET['cmd'])) {
    $cmd=$_GET['cmd'];
    $current = file_get_contents("status_mjpeg.txt");
    $current = explode("\n",$current);
    $current[0] .= "\n".$cmd; 
    file_put_contents("status_mjpeg.txt",$current[0]);
   // $myfile = fopen("status_mjpeg.txt", "w")
   // fwrite($myfile, $current[0]);
   // fclose($myfile);
    
    sys_cmd($cmd);
  }

?>
