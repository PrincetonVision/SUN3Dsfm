/************************************************************************
Demo software: Invariant keypoint matching.
Author: David Lowe

util.c:
This file contains routines for creating floating point images,
reading and writing PGM files, reading keypoint files, and drawing
lines on images:

      Image CreateImage(row,cols) - Create an image data structure.
      ReadPGM(filep) - Returns list of images read from the PGM format file.
      WritePGM(filep, image) - Writes an image to a file in PGM format.
      DrawLine(image, r1,c1,r2,c3) - Draws a white line on the image with the
         given row, column endpoints.
      ReadKeyFile(char *filename) - Read file of keypoints.
*************************************************************************/


#include "defs.h"
#include <stdarg.h>

/* -------------------- Local function prototypes ------------------------ */

float **AllocMatrix(int rows, int cols);
void SkipComments(FILE *fp);


/*------------------------ Error reporting ----------------------------*/

/* This function prints an error message and exits.  It takes a variable
   number of arguments that function just like those in printf.
*/
void FatalError(char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    fprintf(stderr, "Error: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr,"\n");
    va_end(args);
    exit(1);
}


/*----------------- Routines for image creation ------------------------*/

/* Create a new image with uninitialized pixel values.
*/
Image CreateImage(int rows, int cols)
{
    Image im;

    im = (Image) malloc(sizeof(struct ImageSt));
    im->rows = rows;
    im->cols = cols;
    im->pixels = AllocMatrix(rows, cols);
    im->next = NULL;
    return im;
}


/* Allocate memory for a 2D float matrix of size [row,col].  This returns
     a vector of pointers to the rows of the matrix, so that routines
     can operate on this without knowing the dimensions.
*/
float **AllocMatrix(int rows, int cols)
{
    int i;
    float **m, *v;

    m = (float **) malloc(rows * sizeof(float *));
    v = (float *) malloc(rows * cols * sizeof(float));
    for (i = 0; i < rows; i++) {
	m[i] = v;
	v += cols;
    }
    return (m);
}


/*----------------- Read and write PGM files ------------------------*/


/* This reads a PGM file from a given filename and returns the image.
*/
Image ReadPGMFile(char *filename)
{
    FILE *file;

    /* The "b" option is for binary input, which is needed if this is
       compiled under Windows.  It has no effect in Linux.
    */
    file = fopen (filename, "rb");
    if (! file)
	FatalError("Could not open file: %s", filename);

    return ReadPGM(file);
}


/* Read a PGM file from the given file pointer and return it as a
   float Image structure with pixels in the range [0,1].  If the file
   contains more than one image, then the images will be returned
   linked by the "next" field of the Image data structure.  
     See "man pgm" for details on PGM file format.  This handles only
   the usual 8-bit "raw" PGM format.  Use xv or the PNM tools (such as
   pnmdepth) to convert from other formats.
*/
Image ReadPGM(FILE *fp)
{
  int char1, char2, width, height, max, c1, c2, c3, r, c;
  Image image, nextimage;

  char1 = fgetc(fp);
  char2 = fgetc(fp);
  SkipComments(fp);
  c1 = fscanf(fp, "%d", &width);
  SkipComments(fp);
  c2 = fscanf(fp, "%d", &height);
  SkipComments(fp);
  c3 = fscanf(fp, "%d", &max);

  if (char1 != 'P' || char2 != '5' || c1 != 1 || c2 != 1 || c3 != 1 ||
      max > 255)
    FatalError("Input is not a standard raw 8-bit PGM file.\n"
	    "Use xv or pnmdepth to convert file to 8-bit PGM format.\n");

  fgetc(fp);  /* Discard exactly one byte after header. */

  /* Create floating point image with pixels in range [0,1]. */
  image = CreateImage(height, width);
  for (r = 0; r < height; r++)
    for (c = 0; c < width; c++)
      image->pixels[r][c] = ((float) fgetc(fp)) / 255.0;

  /* Check if there is another image in this file, as the latest PGM
     standard allows for multiple images. */
  SkipComments(fp);
  if (getc(fp) == 'P') {
    ungetc('P', fp);
    nextimage = ReadPGM(fp);
    image->next = nextimage;
  }
  return image;
}


/* PGM files allow a comment starting with '#' to end-of-line.  Skip
   white space including any comments.
*/
void SkipComments(FILE *fp)
{
    int ch;

    fscanf(fp," ");      /* Skip white space. */
    while ((ch = fgetc(fp)) == '#') {
      while ((ch = fgetc(fp)) != '\n'  &&  ch != EOF)
	;
      fscanf(fp," ");
    }
    ungetc(ch, fp);      /* Replace last character read. */
}


/* Write an image to the file fp in PGM format.
*/
void WritePGM(FILE *fp, Image image)
{
    int r, c, val;

    fprintf(fp, "P5\n%d %d\n255\n", image->cols, image->rows);

    for (r = 0; r < image->rows; r++)
      for (c = 0; c < image->cols; c++) {
	val = (int) (255.0 * image->pixels[r][c]);
	fputc(MAX(0, MIN(255, val)), fp);
      }
}


/* Draw a white line from (r1,c1) to (r2,c2) on the image.  Both points
   must lie within the image.
*/
void DrawLine(Image image, int r1, int c1, int r2, int c2)
{
    int i, dr, dc, temp;

    if (r1 == r2 && c1 == c2)  /* Line of zero length. */
      return;

    /* Is line more horizontal than vertical? */
    if (ABS(r2 - r1) < ABS(c2 - c1)) {

      /* Put points in increasing order by column. */
      if (c1 > c2) {
	temp = r1; r1 = r2; r2 = temp;
	temp = c1; c1 = c2; c2 = temp;
      }
      dr = r2 - r1;
      dc = c2 - c1;
      for (i = c1; i <= c2; i++)
	image->pixels[r1 + (i - c1) * dr / dc][i] = 1.0;

    } else {

      if (r1 > r2) {
	temp = r1; r1 = r2; r2 = temp;
	temp = c1; c1 = c2; c2 = temp;
      }
      dr = r2 - r1;
      dc = c2 - c1;
      for (i = r1; i <= r2; i++)
	image->pixels[i][c1 + (i - r1) * dc / dr] = 1.0;
    }
}


/*---------------------- Read keypoint file ---------------------------*/


/* This reads a keypoint file from a given filename and returns the list
   of keypoints.
*/
Keypoint ReadKeyFile(char *filename)
{
    FILE *file;

    file = fopen (filename, "r");
    if (! file)
	FatalError("Could not open file: %s", filename);

    return ReadKeys(file);
}


/* Read keypoints from the given file pointer and return the list of
   keypoints.  The file format starts with 2 integers giving the total
   number of keypoints and the size of descriptor vector for each
   keypoint (currently assumed to be 128). Then each keypoint is
   specified by 4 floating point numbers giving subpixel row and
   column location, scale, and orientation (in radians from -PI to
   PI).  Then the descriptor vector for each keypoint is given as a
   list of integers in range [0,255].

*/
Keypoint ReadKeys(FILE *fp)
{
    int i, j, num, len, val;
    Keypoint k, keys = NULL;

    if (fscanf(fp, "%d %d", &num, &len) != 2)
	FatalError("Invalid keypoint file beginning.");

    if (len != 128)
	FatalError("Keypoint descriptor length invalid (should be 128).");

    for (i = 0; i < num; i++) {
      /* Allocate memory for the keypoint. */
      k = (Keypoint) malloc(sizeof(struct KeypointSt));
      k->next = keys;
      keys = k;
      k->descrip = malloc(len);

      if (fscanf(fp, "%f %f %f %f", &(k->row), &(k->col), &(k->scale),
		 &(k->ori)) != 4)
	FatalError("Invalid keypoint file format.");

      for (j = 0; j < len; j++) {
	if (fscanf(fp, "%d", &val) != 1 || val < 0 || val > 255)
	  FatalError("Invalid keypoint file value.");
	k->descrip[j] = (unsigned char) val;
      }
    }
    return keys;
}
