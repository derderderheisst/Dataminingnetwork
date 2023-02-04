/** @file
 *		Description of the file content. 
 * 		This is a link to the section @ref s1.
 *
 *
 * @author
 *		Samuele Zoppi <samuele.zoppi@tum.de>
 */

/**
 * @brief Very important static value
 *
 */
#define MY_VALUE 0

/**
 * @brief Very important integer global variable
 *
 */
int my_int;

/**
 * @brief		Starts the project
 * @param day	Intial day of the project
 * @param title Title of the project 
 * @retval		Zero if the project could be started, non-zero otherwise
 *
 * 				Complete detailed description of the function...
 *
 */
int project_start(int day, char* title){
	if(day==MY_VALUE) /// @warning Check the correctness of the value
		return 0;
	else		
		return 1;
}
