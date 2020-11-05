#define _CRT_SECURE_NO_WARNINGS

#include<stdio.h>
#include<math.h>
#include <time.h> 
#define PI 3.141592
float getRadian(float num) {
    return num * (PI / 180);
}
int main() {
    int count = 0;
    FILE* fp = fopen("armangle.csv", "w");
    time_t start = time(NULL);
    float one;

    float two;
    float three;
    float four;
    float length;
    float x_value;
    float y_value;
    for (float c = 0;c <=30;c += 0.1) {
        for (float i = 0; i <= 135; i = i + 0.1) {
            for (float j = -90; j <= 90; j = j + 0.1) {

                for (float k = -90; k <= 90; k = k + 0.1) {

                    one = getRadian(c);
                    two = getRadian(i);
                    three = getRadian(j);
                    four = getRadian(k);
                    length = (cos(one)) * ((9.5 * cos(two)) + (12.1 * cos(two + three)) + (12 * cos(two + three + four)));
                    //printf("%f\n", sin(90));
                    //printf("%f\n", length);


                    if (length < 14.8 || length>15.02) {
                        printf("c:%f j:%f k:%fcount:%d\n", c, j,k, count);
                        /*if (c < 89)
                            c += 2;*/
                            /*if (j < 70) {
                                j += 2;
                                k = -90;
                            }*/
                            /*if (length < ) {
                                j += 1;
                                k = -90;
                            }*/
                        if (length < 13&& length > 15.2) {
                            if (k < 90)
                            {
                                if (k <= 80) {
                                    k += 20;
                                }
                                else {
                                    j += 1;
                                    k = -90;
                                }
                            }
                            
                        }
                        else {
                            k += 0.5;
                        }



                    }
                    
                    if (length >= 15.0 && length <= 15.003) {
                        printf("up\n");
                        x_value = (sin(one)) * ((9.5 * cos(two)) + (12.1 * cos(two + three)) + (12 * cos(two + three + four)));
                        if (x_value >= -7.5 && x_value <= 7.5 && (int)(1000 * x_value) % 150 == 0) {
                            y_value = (cos(one)) * ((9.5 * sin(two)) + (12.1 * sin(two + three)) + (12 * sin(two + three + four)));
                            if (y_value >= 0 && y_value <= 15 && (int)(y_value * 1000) % 150 == 0) {
                                printf("%d\n", ++count);
                                fprintf(fp, "%f,%f,%f,%f,%f,%f \n", x_value, y_value, c, i, j, k);
                            }
                        }
                    }
                }
            }
        }

    }
    fclose(fp);
}