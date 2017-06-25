#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
int main(int argc, char *argv[]){

	//---------------------------------
	//----- CREATE THE GTK WINDOW -----
	//---------------------------------
	GtkWidget *MainWindow;
	
	gtk_init(&argc, &argv);
	
	MainWindow= gtk_window_new(GTK_WINDOW_TOPLEVEL); 		//GTK_WINDOW_TOPLEVEL = Has a titlebar and border, managed by the window manager. 
	gtk_window_set_title(GTK_WINDOW(MainWindow), "My Application");
	gtk_window_set_default_size(GTK_WINDOW(MainWindow), 400, 300);		//Size of the the client area (excluding the additional areas provided by the window manager)
	gtk_window_set_position(GTK_WINDOW(MainWindow), GTK_WIN_POS_CENTER);
	gtk_widget_show_all(MainWindow);

	//Close the application if the x button is pressed if ALT+F4 is used
	g_signal_connect(G_OBJECT(MainWindow), "destroy", G_CALLBACK(gtk_main_quit), NULL);
	
	//GtkWidget *text = gtk_text_new(NULL,NULL);
	
	//gtk_text_insert(GTK_TEXT (text),
	//----- ENTER THE GTK MAIN LOOP -----
	gtk_main();		//Enter the GTK+ main loop until the application closes.

	return 0;
}
