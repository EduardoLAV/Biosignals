#!/usr/bin/php
<?php

function prompt_silent($prompt = "Enter Password:") {
  if (preg_match('/^win/i', PHP_OS)) {
    $vbscript = sys_get_temp_dir() . 'prompt_password.vbs';
    file_put_contents(
      $vbscript, 'wscript.echo(InputBox("'
      . addslashes($prompt)
      . '", "", "password here"))');
    $command = "cscript //nologo " . escapeshellarg($vbscript);
    $password = rtrim(shell_exec($command));
    unlink($vbscript);
    return $password;
  } else {
    $command = "/usr/bin/env bash -c 'echo OK'";
    if (rtrim(shell_exec($command)) !== 'OK') {
      trigger_error("Can't invoke bash");
      return;
    }
    $command = "/usr/bin/env bash -c 'read -s -p \""
      . addslashes($prompt)
      . "\" mypassword && echo \$mypassword'";
    $password = rtrim(shell_exec($command));
    fprintf(STDERR, "\n");
    return $password;
  }
}

if($argc < 3){
	die("Usage: ./github-export.php %USERNAME% %REPOSITORY%");
}

$username = $argv[1];

$password = prompt_silent("Your password: ");

$repository = $argv[2];

$ch = curl_init("https://api.github.com/repos/$repository/issues?status=open&per_page=1000");
curl_setopt($ch, CURLOPT_USERPWD, $username . ":" . $password); 
curl_setopt($ch, CURLOPT_USERAGENT, 'asmisha parser');
curl_setopt($ch, CURLOPT_RETURNTRANSFER, 1);
$issues = json_decode(curl_exec($ch));

//$fp = fopen('out.csv', 'w');
$fp = STDOUT;
fputcsv($fp, array(
	'Milestone',
	'ID',
	'State',
	'Title',
	'Description'
));

foreach($issues as $i){
	fputcsv($fp, array(
		@$i->milestone->title,
		@$i->number,
		@$i->state,
		@$i->title,
		@$i->body
	));
}

//fclose($fp);