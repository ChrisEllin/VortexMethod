#ifndef MainField_H
#define MainField_H

#include <QObject>
#include <QWidget>

#include <QTimer>
#include <QMouseEvent>

#include <QVector2D>
#include <QVector3D>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShader>
#include <QOpenGLTexture>

struct Vertex
{
    QVector4D coord;
    QVector3D color;
};

enum t_draw {
    Arrow,
    Points
};

struct SArrow
{
public:
    QVector3D topCenter;
    QVector3D botCenter;
    QVector3D middle;
    QVector <QVector3D> arrowHead;
    enum vort_type {Grid, Free} vortonType;
};

class MainField : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

private:
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer axisVertBuffer;
    QOpenGLBuffer axisIndexBuffer;

    QOpenGLShaderProgram *faceProgram;
    QOpenGLShaderProgram *blackBorderProgram;
    QOpenGLShaderProgram *lineProgram;
    QOpenGLShaderProgram *shadowmapProgram;
    QOpenGLShaderProgram *textureProgram;
    QOpenGLShaderProgram *textureDepthProgram;

    QOpenGLTexture* lightTexture;

    QMatrix4x4 matrixCamera;
    QMatrix4x4 matrixWorld;
    QMatrix4x4 matrixRotation;
    QMatrix4x4 matrixTranslation;
    QMatrix4x4 matrixProjection;
    QImage out;
    bool render;
    bool shadowRender;
    t_draw drawFreeType; /// способ вывода свободных вортонов
    t_draw drawGridType; /// способ вывода вортонов, принадлежащих сетке

//    QMatrix4x4 fixedMatrix;
    double zoom;

    float boundaryRange;

    QTimer timer;

    bool turning;
    bool moving;
    QPoint mousePositionStart;

    QVector <SArrow> grid;
    QVector <SArrow> gridGA;
    QVector <SArrow> freeVortons;
    QVector3D posCenter;
    QVector3D posEye;
    QVector3D posUp;
    QVector3D posMoved;
    Vertex marker[4];

    void drawGrid();
    void drawGridGA();
    void drawFreeVortons();
    void drawCylinder();
    void drawSphere(double r);
    void drawAxis();
    void drawMarker();
    void updateMarker();
    void updateBoundaries(QVector3D vector);
    QMatrix4x4 setMatrix();
    void setZoom(double zoom);
public:
    double radius;
    double axisLength;
    enum shape {Cylinder, Sphere, Other, None} Shape;
    explicit MainField(QWidget *parent = nullptr);
    ~MainField();

    QColor backgroundColor;

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mousePressEvent  (QMouseEvent* e);
    void mouseMoveEvent   (QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);

signals:
    void zoomChanged(double zoom);
    void sendScreenshot(QImage& screenshot);

    void testStuff(QString& stuff);

public slots:
    void setPlaneXY();
    void setPlaneYX();
    void setPlaneXZ();
    void setPlaneZX();
    void setPlaneYZ();
    void setPlaneZY();

    void zoomIn();
    void zoomOut();
    void resetPlane();

    void animate();

    void addVorton(QVector3D botCenter, QVector3D topCenter, SArrow::vort_type drawFreeType);
    void addVorton(QVector3D botCenter, QVector3D topCenter);
    void addVortonGA(QVector3D botCenter, QVector3D topCenter, SArrow::vort_type drawType);
    void addVortonGA(QVector3D botCenter, QVector3D topCenter);
    void clearCylinders(int n = -1);

    void setRadius(double radius);
    void setHeight(double axisLength);

    void changeModel(MainField::shape newShape);

    void updateColors(QColor backColor);
    void updateFreeVortonsMode(bool asPoints);
    void updateGridVortonsMode(bool asPoints);
    void updateZoom(double zoom);
};

#endif // MainField_H

