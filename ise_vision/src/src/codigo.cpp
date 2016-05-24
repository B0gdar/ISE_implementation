#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <list>

/*************************************************/
////////////////////THRESHOLDS/////////////////////
/*************************************************/

//Para eliminacion de contornos de tamaño menor a 20
#define THRESHOLD 20

//Para la obtencion de contornos y filtrado. Eliminación de contornos con centros demasiado cercanos
#define THRE_EPSILON 0.1
#define AREA_MIN 1000
#define CENTER_THRES 1

//Para filtro Gaussiano
#define BINARY 120

//Tamaño de la imagen AruCo a mostrar. Division de la Grid
#define TAM_IM 400
#define DIVISION_SUB 7

using namespace cv;
using namespace std;

/*************************************************/
////////////////Global variables///////////////////
/*************************************************/

double areaContEsquina = 0;

int codigo_Check = 0;
//Control de el numero de imagenesArUco representadas
int contador_pic = 0;
//Bandera para error de lectura ArUco
int flag_error = 0;
//Vector para guardar el código de cada ArUco marker
char code[2*(DIVISION_SUB - 2) + 1];

char code_final[2*(DIVISION_SUB - 2) + 1];

/*************************************************/
////////////////// Funciones //////////////////////
/*************************************************/

//Funcion que analiza si el subcuadro del grid de ArUco predomina el negro o el blanco
//y devuelve un 1 si negro o 0 si blanco
int SubMatAnalysis( Mat Im, int a1, int b1, int a2, int b2, int divC, int divR);

//Analiza la imagen ArUco obtenida, la divide en submatrices, establece si la submatriz es negra o blanca
//guardando los resultados de llamar a SubMatAnalysis en un vector, y leyendo el código único de la 
//imagen ArUco obtenida, devolviendo dicho código como un char en el return
vector<int > arUco_analysis ( Mat ImageIn );

//Coloca las esquinas de un contorno cerrado en sentido antihorario entorno al centro de masas del contorno
void sortCorners(std::vector<cv::Point>& corners, cv::Point2f center);

//Funcion que calcula el angulo entre 3 puntos
double angle( Point pt1, Point pt2, Point pt0 );

void lectura_lista();

list <string> nombres;


/*************************************************/
///////////////Programa Principal//////////////////
/*************************************************/

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/bebop/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
//	std::cout << "se abre";
		
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    lectura_lista();
    Mat image, cdst, cdst_out;
  
    cvtColor(cv_ptr->image, image,CV_BGR2GRAY);
	cdst_out = cv_ptr->image.clone(); 
	cdst = cv_ptr->image.clone(); 

	/*************************************************/
	///////////////Filtrado de imagen/////////////////
	/*************************************************/

	//Declaración de imágenes para filtrado intermedio
	Mat detected_edges, canny_edges;
	blur(cv_ptr->image, detected_edges, Size(3,3) );	//Primera distorsión
 	Canny(detected_edges,canny_edges, 20, 200, 3);	//Obtenemos las líneas principales de la imágen
 	
	/*************************************************/
	//////////Obtención y filtrado de contornos////////
	/*************************************************/
 
	//Matrices y vectores de puntos (x,y)
	vector<vector<Point> > contours;
	vector<vector<Point> > contours_poly;
	vector<Vec4i> hierarchy;
	//Buscamos los contornos cerrados en la imagen
 	findContours(canny_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


 	//los dibujamos, no es necesario, solo para ajustar los threshold
 	//for( int i = 0; i< contours.size(); i++ ){
 	//    drawContours( cdst, contours, i, Scalar( 0, 255, 0), 1, 8, hierarchy, 0, Point() );
 	//}
 	//imshow("PrePoly", cdst);

	/*************************************************/
	/////////////Poliedrizamos los contornos///////////
	/*************************************************/

 	for( int i = 0; i< contours.size(); i++ ){
		//Parámetro de threshold
		double epsilon = THRE_EPSILON*arcLength(contours[i], true);		
		//Creamos un nuevo vector de contornos ya poliedrizados
		contours_poly.resize(contours.size());
		//Poliedrizamos y guardamos en el nuevo vector
		approxPolyDP( Mat(contours[i]), contours_poly[i], epsilon, true );
		//Los dibujamos, si queremos
		//drawContours( cdst, contours_poly, i, Scalar( 255, 0, 0), 1, 8, hierarchy, 0, Point() );
	 }

	 //Para calcular threshold del proceso, podemos sacar la imagen por pantalla
	 //imshow("postPoly", cdst);

	/********************************************************/
	//Buscamos poliedros de 4 esquinas y eliminamos el resto//
	/********************************************************/

 	//Filtramos los poligonos de 4 esquinas
	int cont = 0;
	//Variables auxiliares empleadas para medir distancia del centro de masas del contorno actual 
	//con el anterior para eliminar así los que esten demasiado juntos
	double cx_ant = 0;
	double cy_ant = 0;

 	for (vector<vector<Point> >::iterator it = contours_poly.begin(); it!=contours_poly.end(); ){
 		
		//CALCULAMOS EL centro de masas y los guardamos en el vector Moments
    		vector<Moments> M( contours_poly.size() );  
		//Se obtienen las cordenadas de los momentos de masas y se guarda en el vector M
    		M[cont] = moments( contours_poly[cont], false );
		//Variables auxiliares para comparar el centro actual con el anterior
    		double cx = (double)(M[cont].m10/M[cont].m00);
    		double cy = (double)(M[cont].m01/M[cont].m00 );
		//Variable auxiliar en la que se guarda la distancia entre centros
    		double distance_center;
		//Calculo la distancia entre ambos centros
    		distance_center =  sqrt( (cx_ant - cx)*(cx_ant - cx) + (cy_ant - cy)*(cy_ant - cy));
		//Si lo necesito, la saco por pantalla
    		//cout << "dist " << distance_center << endl;

    		//FILTRAMOS: eliminaremos a la vez los contornos que no tengan cuatro esquinas, que sean concavos, demasiado junto al anterior
		//y con esquinas formando angulos distintos de aproximadamente 90 grados
    		if( contourArea(contours_poly[cont]) >= AREA_MIN && contours_poly[cont].size() == 4 
		&& isContourConvex(contours_poly[cont]) == true
		&& distance_center >= CENTER_THRES){
			//Guardamos el centro actual en la variable auxiliar de centro anterior
       			cx_ant = cx;
       			cy_ant = cy; 
			//Variable para calcular el angulo
       			double maxCosine = 0;	
			//Calculo de los angulos de las 4 esquinas y guardado del anterior
       			for( int j = 2; j < 5; j++ ){
       				double cosine = fabs(angle(contours_poly[cont][j%4], contours_poly[cont][j-2], contours_poly[cont][j-1]));
       				maxCosine = MAX(maxCosine, cosine);
       			}
			//Si el ángulo máximo es menor de un rango (Aprox 90 y poco en este caso)
       			if( maxCosine < 0.3 ){
				//Los dibujo en la imagen cdst para verlos
       				//drawContours( cdst, contours_poly, cont, Scalar( 0, 0, 255), 1, 8, hierarchy, 0, Point() );
				//Aumento contadores
        			++it;
        			++cont;
       			}
			//Si hay angulos mayores, elimino este contorno del vector de contornos
       			else it = contours_poly.erase(it);
    		}
		//Si no cumple con las condiciones, elimino el contorno del vector de contornos
    		else it = contours_poly.erase(it);
	} 

	//Si necesito afinar, saco la imagen con los contornos filtrados por pantalla
   	//imshow("postSquares", cdst); 
	//Vector de imagenes
   	vector<Mat> pictures;
	//Vector de los códigos de cada imagen ArUco
   	char code_vec[contours_poly.size()][2*(DIVISION_SUB - 2) + 1];
   	//Busca los centros de los recuadros
	//vector de cordenadas de los centros
   	vector<Point2f> center_vec;
	//Vector de coordenadas de corners de los contornos
   	vector<Point2f> corner_vec; 
	//Vector de vectores de coordenadas de corners de los contornos
   	vector<vector<Point2f> > corner_mat; 
	vector<int>  cont_poly_erase;
	//Saca por pantalla el numero de recuadros
   	cout << "Numero de recuadros: " << contours_poly.size() << endl;
	int cont_ok = 0;
	/****************************************************************/
	///Analizamos cada contorno, calculamos su centro, reoordenamos///
	////esquinas, ponemos las imagenes ArUco rectas, las analizamos///
	//extraemos el código de ellas, guardamos el código en un vector//
	//////////de códigos, y lo metemos en la imagen///////////////////
	/****************************************************************/
	//Analizamos cada contorno
	for (int i = 0; i < contours_poly.size(); i++){
		//Variables auxiliares para calcular las esquinas y centros de los contornos
  		Point2f center(0,0);
  		vector<Point2f> corner_vec; 
  		Point2f corner(0,0);
		//Analizamos cada esquina, la guardamos y calculamos el centro del contorno
  		for (int j = 0; j < contours_poly[i].size(); j++){
			//Calculo de centro. Ni idea de como, esto lo he sacado de Internet
  			center.x += contours_poly[i][j].x;
  			center.y += contours_poly[i][j].y;
      		}
   		//Con los datos de las esquinas calculamos las coordenadas del contorno i
   		center.x *= (1. / contours_poly[i].size());
   		center.y *= (1. / contours_poly[i].size());
		//Si lo necesitamos, sacamos los centros por pantalla
   		//cout << "Xc  " << center.x << endl;
   		//cout << "Yc  " << center.y << endl;
		//Ordenamos las esquinas de forma antihoraria
   		sortCorners(contours_poly[i], center);
		
		for(int j = 0; j < contours_poly[i].size(); j++){
			//Guardo la coordenada de la esquina actual
      			corner.x = contours_poly[i][j].x;
      			corner.y = contours_poly[i][j].y;
			//Metemos la esquina en el vector de corners
      			corner_vec.push_back(corner);
		}
		//Metemos las coordenadas del contorno i en el vector de vectores de coordenadas corner_mat
   		corner_mat.push_back(corner_vec); 
		
		/********************************************************/
		/////Obtenemos la imagen ordenada de cada código ArUco////
		/********************************************************/
		
		//Imagen auxiliar donde se guardará la imagen ordenada
		//De tamaño TAM_IMxTAM_IM
   		cv::Mat quad = cv::Mat::zeros(TAM_IM, TAM_IM, CV_8UC3);
		
 		 //Preparamos dicha imagen, para recibir la imagen transformada
  		std::vector<cv::Point2f> quad_pts;
  		quad_pts.push_back(cv::Point2f(0, 0));
  		quad_pts.push_back(cv::Point2f(quad.cols, 0));
		quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
		quad_pts.push_back(cv::Point2f(0, quad.rows));
  		
   		//Con las coordenadas de las esquinas del contorno i, transformamos los pixelex que hay en el 
		//interior de este
   		cv::Mat transmtx = cv::getPerspectiveTransform(corner_mat[i], quad_pts);
   		//Aplicamos la perspectiva sobre cdst_out y guardamos la imagen transformada en quad
   		cv::warpPerspective(cdst_out, quad, transmtx, quad.size());
		//Transformamos la imagen quad a blanco y negro (grises)
   		cvtColor( quad, quad, CV_BGR2GRAY );
		//Aplicamos distorsion gaussiana para filtrar lineas que sobran de la imagen
   		GaussianBlur( quad, quad, Size( 5, 5 ), 0, 0);
		//Aplicamos un filtro binario + otsu (deja la imagen en blanco y negro SOLO (sin grises)
   		threshold( quad, quad, BINARY , 255, 0 + THRESH_OTSU);
		//Volvemos a RGB para poder analizar la imagen pixel a pixel
   		cvtColor( quad, quad, CV_GRAY2BGR );
		//Funcion creada para analizar la imagen ArUco (quad) y devolver el código que guarda
   		vector<int>  codigo = arUco_analysis(quad);
		
		
		/*********************************************************************/
		//Para AJUSTE: creamos un grid en la imagen quad para ver su división//
		/*********************************************************************/
	
		//Anchura de la imagen
		int width=quad.size().width;
		int height=quad.size().height;
		//Pixeles por division. El numero de divisiones es DIVISION_SUB
		int distH = (int)height/DIVISION_SUB;
		int distW = (int)width/DIVISION_SUB; 
		
		//Lineas verticales
		for(int a=0;a<height;a+=distH)
			  cv::line(quad,Point(0,a),Point(width,a),cv::Scalar(255,255,255));
		//Lineas horizonales
		for(int a=0;a<width;a+=distW)
			  cv::line(quad,Point(a,0),Point(a,height),cv::Scalar(255,255,255));
	
		//Nombre de cada imagen quad, importante, ya que si no se cambia este nombre se sobreescriben las fotos
		char Name[6] = "Pic 0";
		Name[4] = Name[4] + i;
		
	
		//La funcion que analiza el arUco code, pone una bandera a 0 si hay error de código, por ejemplo, que
		//El arUco no tenga un recuadro exterior completamente negro
		//Si no hay error
		if(!flag_error){
  			// Se obtiene un iterador al inicio de la lista  
  			list<string>::iterator it = nombres.begin();
   			// Buscamos el elemento "Pedro"
    			while ( it != nombres.end() ){
				if( *it == code ){
					codigo_Check = 1;
					break;
				}
				else{
					codigo_Check = 0;
				}
				it++;			
				
			}
			if( codigo_Check == 1){
				cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" << endl;
				//Escribimos el código en el centro de la imagen ArUco quad
				putText( quad, code , Point(quad.cols/2-quad.cols/4, quad.rows/2), 1, 2, Scalar(0, 0,255), 2, 8);
				//Escribimos en la imagen principal el código en el centro de cada imagen arUco detectada en la imagen principal
				//Metemos el centro calculado en el vector de centros
   				center_vec.push_back(center);
				//Representamos cada esquina en la imagen final
				circle( cdst, corner_vec[0], 2, Scalar(255,0,0), 2, CV_AA);
				circle( cdst, corner_vec[1], 2, Scalar(0,255,0), 2, CV_AA);
				circle( cdst, corner_vec[2], 2, Scalar(0,0,255), 2, CV_AA);
				circle( cdst, corner_vec[3], 2, Scalar(0,0,0), 2, CV_AA);
				//Dibujamos los centros en la imagen final
   				circle( cdst, center, 8, Scalar(255,0,255),8 , CV_AA);
				putText( cdst, code , center, 1, 0.9, Scalar(0, 255, 255), 2, 8);
				//Metemos quad en el vector de imágenes
   				pictures.push_back(quad);
				//Sacamos la imagen arUco quad por pantalla
   				imshow("1" , quad);
				//Guardamos el código obtenido en el vector de códigos
        			strcpy(code_vec[cont_ok],code);
				cont_ok++;
				//Sacamos el código por pantalla
				cout << "Codigo :" << code << endl;
				//+ 1 al contador de imágenes
   				contador_pic++;
			}
			else{
				cont_poly_erase.push_back(i);
			}
   		}
		//Si hay error, reiniciamos la bandera e ignoramos la imagen obtenida.
   		else{
			cont_poly_erase.push_back(i);		
			flag_error = 0;
		}
	}
	vector<vector<Point> >::iterator it_poly = contours_poly.begin();
	for( int i = 0; i < cont_poly_erase.size(); i++){
		it_poly = it_poly + cont_poly_erase[i]; 
		it_poly = contours_poly.erase(it_poly);	
	}
	for( int i = 0; i < contours_poly.size(); i++){
		/*char aux[3]= "  ";
		aux[1] = i + '0';
		putText( cdst, aux , center_vec[i], 1, 0.9, Scalar(0, 255, 255), 2, 8);
		putText( cdst, aux , corner_mat[i][0], 1, 0.9, Scalar(0, 255, 255), 2, 8);*/
		drawContours( cdst, contours_poly, i, Scalar( 0, 0, 255), 1, 8, hierarchy, 0, Point() );
	}
	vector <Point2f> Pairs_vec;
	vector <float> distances_vec;
	vector <int> posicion;
	double max_dist = 0;
	int max_pos = 0;
	Point medio; 
	int medio_out[2];
	int code_vec_size;
	if( contours_poly.size() >= 2 ){
		areaContEsquina = 0;
		int auxc = 0;
		for( int n = 0; n < contours_poly.size() ; n++){
			for( int m = n + 1; m < contours_poly.size(); m++){
				//cout << code_vec[m] << " " << code_vec[n] << " "  << endl ;
				if( strcmp(code_vec[m],code_vec[n]) == 0 ){
					cout << " Iguales "  << endl ;
					Point2f aux;
					aux.x = center_vec[m].x;
					aux.y = center_vec[m].y;
					Pairs_vec.push_back(aux);
					posicion.push_back(m);
					posicion.push_back(n);
					aux.x = center_vec[n].x;
					aux.y = center_vec[n].y;
					Pairs_vec.push_back(aux);
					auxc++;
				}
			}
		}
		for (int i = 0; i < contours_poly.size(); i++) cout << code_vec[i] << endl;
		
		//cout << auxc << " " << contours_poly.size() << " " << center_vec.size() << endl;
		for( int n = 0; n < Pairs_vec.size(); n = n+2){
			float distances;
			distances = sqrt( (Pairs_vec[n].x - Pairs_vec[n+1].x)*(Pairs_vec[n].x - Pairs_vec[n+1].x) + (Pairs_vec[n].y - Pairs_vec[n+1].y)*(Pairs_vec[n].y - Pairs_vec[n+1].y));
			distances_vec.push_back(distances);
			cout << distances << endl;
			if(distances >= max_dist){
				max_dist = distances; 
				max_pos = n;
			}
		}
		//cout << max_dist << " en pos: " << max_pos << endl;
		//cout << distances_vec[0] << " " << distances_vec[1] << " "  << endl ;
		//waitKey(0);
		for( int s = 0; s < 2*(DIVISION_SUB - 2) + 1; s++) code_final[s] = code_vec[max_pos][s];
		for( int n = 0; n < Pairs_vec.size() ; n = n+2){
			if ( n == max_pos ){
				
				line(cdst, Pairs_vec[n], Pairs_vec[n+1], Scalar(255,0,0),3,8,0);
				medio.x = fabs((Pairs_vec[n].x + Pairs_vec[n+1].x))/2;
				medio.y = fabs((Pairs_vec[n].y + Pairs_vec[n+1].y))/2;
				putText( cdst, code_final , medio, 1, 0.9, Scalar(0, 255, 255), 2, 8);
				circle( cdst, medio, 2, Scalar(0,0,0), 2, CV_AA);
				
			}
			else{
				line(cdst, Pairs_vec[n], Pairs_vec[n+1], Scalar(255,255,255),3,8,0);
			}
		}
		medio_out[0] = medio.x - (int)cdst.cols/2;
		medio_out[1] = medio.y - (int)cdst.rows/2;
	}
	else if ( contours_poly.size() == 1){ //ESQUINA
		for( int s = 0; s < 2*(DIVISION_SUB - 2); s++) code_final[s] = code_vec[0][s];
		areaContEsquina = contourArea(contours_poly[0]);
		medio_out[0] = center_vec[0].x - (int)cdst.cols/2;
		medio_out[1] = center_vec[0].y - (int)cdst.rows/2;
	}
	else{
		medio_out[0] = -1;
		medio_out[1] = -1;
		areaContEsquina = -1;
	}	
	// Se obtiene un iterador al inicio de la lista  
  	list<string>::iterator it = nombres.begin();
	int cont_pos = 0;
	int code_out_pos = -1;
	
	// Buscamos el elemento "Pedro"
	while ( it != nombres.end() ){
		if( *it == code_final ){
			code_out_pos = cont_pos;
			break;
		}
		else{
			code_out_pos = -1;
		}
		it++;
		cont_pos++;			
	}
	//Lineas verticales
	cv::line(cdst,Point(0,cdst.rows/2),Point(cdst.cols,cdst.rows/2),cv::Scalar(255,255,255));
	//Lineas horizonales
	cv::line(cdst,Point(cdst.cols/2,0),Point(cdst.cols/2,cdst.rows),cv::Scalar(255,255,255));
	//ELSE ERRORRRRRR*/
	//Sacamos por pantalla las imágenes que necesitemos ver
	cv_ptr->image = cdst.clone();
	imshow("Final", cv_ptr->image);
	//imshow("Lineas detectadas", canny_edges);
	//imshow("Contornos", detected_edges);
 	//OJOOOOO
	//Hay que enviar code_final, medio_out y areaContEsquina
 	waitKey(10) >= 0;
   	    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


//Funcion que analiza si el subcuadro del grid de ArUco predomina el negro o el blanco
//y devuelve un 1 si negro o 0 si blanco
//Se analiza solo el subcuadro obtenido de la division de la imagen en DIVISION_SUBxDIVISION_SUB
int SubMatAnalysis( Mat Im, int a1, int b1, int a2, int b2, int divC, int divR){
	//COntadores para sumar el numero de pixeles negros y blancos respectivamente
	int contNegro = 0;
	int contBlanco = 0;
	//No analizamos el recuadro entero, solo la parte central, para agilizar el procesamiento
	for( int i = a1*divC + ((int)(a2*divC-a1*divC)/3); i <  a2*divC - ((int)(a2*divC-a1*divC)/3) ; i++){
		for( int j = b1*divR + ((int)(b2*divR-b1*divR)/3); j < b2*divR - ((int)(b2*divR-b1*divR)/3); j++){
			//Sacamos el color del pixel actual
			uchar blue = Im.at<Vec3b>(i,j)[0];
			uchar green = Im.at<Vec3b>(i,j)[1];
			uchar red = Im.at<Vec3b>(i,j)[2];
			//Si el color del pixel es negro (con cierto umbral, aunque no hace falta ya que en la imagen	
			//solo hay pixeles negros o blancos) sumamos +1 al contador negro, si no al blanco.
			if(blue <= 20 && green <= 20 && red <= 20){
				contNegro++;
			}
			else contBlanco++;
		}
	}
	//Si el contador negro es mayor, el subcuadro se considerará negro, si no blanco
	if(contNegro > contBlanco){
		return 1;
	}
	else return 0;
}


//Analiza la imagen ArUco obtenida, la divide en submatrices, establece si la submatriz es negra o blanca
//guardando los resultados de llamar a SubMatAnalysis en un vector, y leyendo el código único de la 
//imagen ArUco obtenida, devolviendo dicho código como un char en el return
//En primer lugar se analiza si la imagen esta recuadrada por cuadros negros, si no, la ignoramos.
//Luego vemos si la imagen es todo negra, si lo es, la ignoramos.
//Si no hay error entonces analizamos el codigo arUco. Se desprecia el recuadro negro exterior, y 
//se analiza el cuadro interior restante. Cada fila corresponde a un codigo hexadecimal de dos dígitos.
//El código de la imagen se guarda en un vector code. Los pixel, obtenidos de la llamada a la funcion anterior, 
//se guardan en un vector que se devuelve como parámetros.

vector<int > arUco_analysis ( Mat ImageIn ){
	//Variable empleada para saber si es un cuadrado completamente negro. Si no hay ningun blanco al acabar el análisis, da ERROR
    	int contBlancos = 0;
	//Variable auxiliar para sacar el código en Hex
    	int aux = 0;
	//Contadores para pasar el codigo ArUco a hexadecimal. Cada fila corresponde a dos letras.
    	int cont_vec = 0;
    	int cont_hex = 0;
	//Vector auxiliar que se devolvera como parámetro
    	vector<int> outputV;   
	//Variables auxiliares para el análisis
    	int pixel;
	//Tamaño de la imagen
    	int pixelCol = ImageIn.cols;
    	int pixelRows = ImageIn.rows;
	//Como cuando se hizo la grid, establecemos el numero de pixeles por division
    	int divisionCols = pixelCol/DIVISION_SUB;
    	int divisionRows = pixelRows/DIVISION_SUB;

	//analizamos cada subcuadro. DIVISION_SUBxDIVISION_SUB en total
    	for( int i = 0; i < DIVISION_SUB ; i++){
		//Reiniciamos las auxiliares en cada fila
		aux=0;
		cont_hex = 0;
		//Vamos fila a fila
		for( int j = 0; j < DIVISION_SUB ; j++){
			//Analizamos el subcuadro y obtenemos el color predominante - negro (1) o blanco(0)
			pixel = SubMatAnalysis(ImageIn, i, j, i+1, j+1, divisionCols, divisionRows);
			//Si blanco, sumamos el contador de blancos para filtrar imagenes completamente negras
			if(pixel == 0) contBlancos++;
			//Sacamos por pantalla para representar la matriz de 1s y 0s			
			cout << "  " << pixel;
			//Metemos el pixel en el vector de pixeles
			outputV.push_back(pixel);
			//Si estamos dentro del recuadro negro exterior, y no en el 
			if( i >= 1 && j>= 1 && j<DIVISION_SUB - 1 && i<DIVISION_SUB - 1){ 
				//Pasamos a hexadecimal la fila
				aux = aux + pixel*pow(2,cont_hex);
				cont_hex++;
				if ( j == 4 ){	
					if( aux > 9){
						code[cont_vec] = 'A' + aux - 10;
					}
					else{
						code[cont_vec] = '0' + aux;
					}
					cont_vec++;
					cont_hex = 0;
					aux = 0;
				}
				if ( j == (DIVISION_SUB - 2 )){	
					if( aux > 9){
						code[cont_vec] = 'A' + aux - 10;
					}
					else{
						code[cont_vec] = '0' + aux;
					}
					cont_vec++;
					cont_hex = 0;
					aux = 0;
				}
			}
			//Si el recuadro exterior no es completamente negro -> ERROR
			if ( (i == 0 || i == (DIVISION_SUB - 1) || j == 0 || j == (DIVISION_SUB - 1)) && pixel == 0) { 
				cout << "ERROR";flag_error = 1;
			}     
		} 
		//Salto de linea paras representar la matriz
		cout << " " << endl;
	}
	//Si son todos los recuadros negros -> ERROR
	if ( contBlancos == 0) { 
		cout << "ERROR";flag_error = 1;
	}   
    	//Sacamos el vector de pixeles como parametro
	return outputV;
}

//Coloca las esquinas de un contorno cerrado en sentido antihorario entorno al centro de masas del contorno
//Este lo encontre por internet, no es mio.
void sortCorners(std::vector<cv::Point>& corners, cv::Point2f center)
{
    std::vector<cv::Point> top, bot;

    for (int i = 0; i < corners.size(); i++)
    {   
        if ((float)corners[i].y < center.y){
            top.push_back(corners[i]);
        }
        else{
            bot.push_back(corners[i]);
        }
    }

    cv::Point tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point br = bot[0].x > bot[1].x ? bot[0] : bot[1];

    corners.clear();
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);
}


//Funcion que calcula el angulo entre 3 puntos. Bueno el angulo no, el arcsen
double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void lectura_lista(){
	// Abre un fichero de entrada
  	std::ifstream In("/home/david/ensayo_ws/src/roscv/src/Codigos.txt");

	string Read;
	if (In.is_open())
  	{
	//bucle para leer todo el contenido e ir metiéndolo en una lista
		while( Read != "End"){
			getline(In,Read);
			nombres.push_back(Read);
		}
		In.close();
  	}
 	else cout << "Unable to open file";
}
	
	//Leemos su contenido*/
