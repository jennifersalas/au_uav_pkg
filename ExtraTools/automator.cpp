/*
automator
This is a program to automatically startup several tests for AU_UAV_ROS evaluator when the
inputs are enumerated in another .txt file.

Compiled with G++.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <algorithm>
#include <string.h>
#include <sstream> //for ostringstream
#include <sys/wait.h>

//normal time + 30 as buffer time
#define SIMULATION_TIME 600
#define BUFFER_TIME 30
#define SLEEP_TIME (SIMULATION_TIME+SIMULATION_TIME)
#define OUTPUT_ADDITION "_test\n" //the '\n' is CRITICAL
#define LENGTH_OF_EXTENSION 7 //".course" has 7 characters
#define CAALGORITHM	"RIPNA"

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
NOTE: Copied from elsewhere
*/
bool isBlankLine(char str[])
{
	for(int i = 0; i < strlen(str); i++)
	{
		switch(str[i])
		{
			case ' ':
			case '\n':
			case '\t':
			{
				//keep checking
				break;
			}
			default:
			{
				//not a blank line character
				return false;
				break;
			}
		}
	}
	
	//we made it here, must be blank
	return true;
}

int main()
{
	//get the filename
	char filename[256];
	printf("Enter the file with all the courses in it:");
	scanf("%s", filename);
	
	//open the file
	FILE *fp;
	fp = fopen(filename, "r");
	
	//check for a good open
	if(fp != NULL)
	{
		char buffer[256];
		std::string timeString = "60";
		//while we have something in the file
		while(fgets(buffer, sizeof(buffer), fp))
		{	//check for useless lines
			if(buffer[0] == '#' || isBlankLine(buffer))
			{
				//this line is a comment
				continue;
			}
			//construct our strings to send
			std::string courseString = std::string(buffer);
			std::string myStr = std::string(buffer);
			myStr = myStr.substr(0, myStr.size() - LENGTH_OF_EXTENSION - 1);
			myStr = myStr + OUTPUT_ADDITION;
			
				
			//child process
			char syscall[256];
			const char *outputFile;
			const char *courseFile;
			courseString = courseString.substr(0,courseString.length()-1);
			courseFile = courseString.c_str();
			outputFile = myStr.substr(0,myStr.length()-1).c_str();

			printf("courseFile: %s\n", courseFile);
			printf("outputFile: %s\n", outputFile);

			sprintf(syscall, "roslaunch au_uav_ros evaluation.launch GUI:=false outputFileParam:=%s courseFileParam:=%s XBeeIO:=false CAAlgorithm:=%s", outputFile, courseFile, CAALGORITHM);
			printf("syscall: %s\n", syscall);
			system(syscall);

			sleep(SLEEP_TIME);			
		}
		
		fclose(fp);
	}
	else
	{
		printf("ERROR: Bad file name\n");
	}
	
	return 0;
}
