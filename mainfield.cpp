#include "mainfield.h"
#include <QtMath>
#include <QOpenGLFramebufferObject>

const int FARSIDE = 4;

QMatrix4x4 biasMatrix (
        0.5, 0.0, 0.0, 0.0,
        0.0, 0.5, 0.0, 0.0,
        0.0, 0.0, 0.5, 0.0,
        0.5, 0.5, 0.5, 1.0
        );

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360;
    while (angle > 360)
        angle -= 360;
}

void MainField::setZoom(double zoom)
{
    float perspective = zoom * 60.0f;
    matrixProjection.setToIdentity();
//    matrixProjection.perspective(perspective, (float)width()/height(), 0.1f, 100.0f);
    matrixProjection.perspective(perspective, static_cast<float>(width())/static_cast<float>(height()), 0.1f, 100.0f);
    this->zoom = zoom;
}

QMatrix4x4 MainField::setMatrix()
{
    return matrixProjection*matrixTranslation*matrixCamera*matrixRotation*matrixWorld;
}

const QColor MARKER_COLOR = Qt::red;

MainField::MainField(QWidget *parent) :
    QOpenGLWidget(parent), axisIndexBuffer(QOpenGLBuffer::IndexBuffer), axisVertBuffer()
{
    setMouseTracking(true);
    turning = false;
    zoom = 1.0;
    emit zoomChanged(zoom * 100.0);

    faceProgram = NULL;
    blackBorderProgram = NULL;
    lineProgram = NULL;
    Shape = MainField::Sphere;
    drawFreeType = Points;
    drawGridType = Arrow;
    radius = 1.0;

    for (int i = 0; i < 4; i++)
    {
        marker[i].color[0] = QColor(MARKER_COLOR).redF();
        marker[i].color[1] = QColor(MARKER_COLOR).greenF();
        marker[i].color[2] = QColor(MARKER_COLOR).blueF();
    }

    connect (&timer, SIGNAL(timeout()), this, SLOT(animate()));
}

void MainField::animate ()
{
    matrixCamera.rotate (3.6, 0, 1, 0);
    update();
}

MainField::~MainField()
{
    makeCurrent();

//    delete m_texture;
//    delete m_shader;
    if (faceProgram)
        delete faceProgram;
    if (blackBorderProgram)
        delete blackBorderProgram;
    if (lineProgram)
        delete lineProgram;

    doneCurrent();
}

void MainField::initializeGL()
{
    radius = 1.0;
    axisLength = 1.0;
    initializeOpenGLFunctions();
    vao.create();
    glEnable(GL_ARB_separate_shader_objects);
    glEnable(GL_PROGRAM_POINT_SIZE);
//    glEnable(GL_MULTISAMPLE);
    glClearColor (0.9, 0.9, 0.9, 1.0);

    // Заполняем координаты осей и сетки
//    Vertex axisBufferData[85];
    Vertex axisBufferData[4];

    axisBufferData[0].coord = QVector3D(0, 0, 0);
    axisBufferData[0].color = QVector3D(1.0, 102.0/255.0 , 0.0);
    axisBufferData[1].coord = QVector3D(FARSIDE, 0, 0);
    axisBufferData[1].color = QVector3D(QColor(Qt::yellow).red()/255.0, QColor(Qt::yellow).green()/255.0, QColor(Qt::yellow).blue()/255.0);
    axisBufferData[2].coord = QVector3D(0, FARSIDE, 0);
    axisBufferData[2].color = QVector3D(QColor(Qt::magenta).red()/255.0, QColor(Qt::magenta).green()/255.0, QColor(Qt::magenta).blue()/255.0);
    axisBufferData[3].coord = QVector3D(0, 0, FARSIDE);
    axisBufferData[3].color = QVector3D(QColor(Qt::cyan).red()/255.0, QColor(Qt::cyan).green()/255.0, QColor(Qt::cyan).blue()/255.0);

//    const int AXIS_COUNT = 9;


//    /*
//     *  Оси пронумерованы вот так:
//     *       __8__
//     *     /|     |
//     *    3 1     7
//     *   /  |     |
//     *  |   |__5__|
//     *  0  /      /
//     *  | 2      4
//     *  |/__6___/
//     *
//     *
//     *           /
//     *         10
//     *   ___9__/
//     *         |
//     *        11
//     *         |
//     *         |
//     */

//     /*       _
//      *     /|
//      *    / |
//      *   |  |_
//      *   | /
//      *   |/_
//      */
//    int i = 0;
//    for (i; i < 2*AXIS_COUNT; i++)
//    {
//        axisBufferData[i + 4].color[0] = 1;
//        axisBufferData[i + 4].coord[0] = static_cast<double>(-FARSIDE);
//        axisBufferData[i + 4].coord[2] = (i < AXIS_COUNT) ? static_cast<double>(-FARSIDE) : static_cast<double>(FARSIDE);
//        axisBufferData[i + 4].coord[1] = ((float)(i % AXIS_COUNT)/(AXIS_COUNT - 1))*2*static_cast<double>(FARSIDE) - static_cast<double>(FARSIDE);
//    }
//    for (i;  i < 4*AXIS_COUNT;i++)
//    {
//        axisBufferData[i + 4].color[0] = 1;
//        axisBufferData[i + 4].coord[0] = static_cast<double>(-FARSIDE);
//        axisBufferData[i + 4].coord[1] = (i < 3*AXIS_COUNT) ? static_cast<double>(-FARSIDE) : static_cast<double>(FARSIDE);
//        axisBufferData[i + 4].coord[2] = ((float)(i % AXIS_COUNT)/(AXIS_COUNT - 1))*2*static_cast<double>(FARSIDE) - static_cast<double>(FARSIDE);
//    }

//     /*      |_____|
//      *           /
//      *   |_____|/
//      */
//    for (i; i < 5*AXIS_COUNT; i++)
//    {
//        axisBufferData[i + 4].color[0] = 1;
//        axisBufferData[i + 4].coord[1] = static_cast<double>(-FARSIDE);
//        axisBufferData[i + 4].coord[0] = static_cast<double>(FARSIDE);
//        axisBufferData[i + 4].coord[2] = ((float)(i % AXIS_COUNT)/(AXIS_COUNT - 1))*2*static_cast<double>(FARSIDE) - static_cast<double>(FARSIDE);
//    }
//    for (i;  i < 7*AXIS_COUNT;i++)
//    {
//        axisBufferData[i + 4].color[0] = 1;
//        axisBufferData[i + 4].coord[1] = static_cast<double>(-FARSIDE);
//        axisBufferData[i + 4].coord[2] = (i < 6*AXIS_COUNT) ? static_cast<double>(-FARSIDE) : static_cast<double>(FARSIDE);
//        axisBufferData[i + 4].coord[0] = ((float)(i % AXIS_COUNT)/(AXIS_COUNT - 1))*2*static_cast<double>(FARSIDE) - static_cast<double>(FARSIDE);
//    }

//     /*
//      *     _____
//      *          |
//      *          |
//      *          |
//      */
//     for (i; i < 8*AXIS_COUNT; i++)
//     {
//         axisBufferData[i + 4].color[0] = 1;
//         axisBufferData[i + 4].coord[0] = static_cast<double>(FARSIDE);
//         axisBufferData[i + 4].coord[2] = static_cast<double>(-FARSIDE);
//         axisBufferData[i + 4].coord[1] = ((float)(i % AXIS_COUNT)/(AXIS_COUNT - 1))*2*static_cast<double>(FARSIDE) - static_cast<double>(FARSIDE);
//     }
//     for (i;  i < 9*AXIS_COUNT;i++)
//     {
//         axisBufferData[i + 4].color[0] = 1;
//         axisBufferData[i + 4].coord[1] = static_cast<double>(FARSIDE);
//         axisBufferData[i + 4].coord[2] = static_cast<double>(-FARSIDE);
//         axisBufferData[i + 4].coord[0] = ((float)(i % AXIS_COUNT)/(AXIS_COUNT - 1))*2*static_cast<double>(FARSIDE) - static_cast<double>(FARSIDE);
//     }

//     // Нет 3 рёбер, которые не видны из точки (1, 1, 1) (9, 10 и 11)

//     // Индексы осей
//    GLushort axisIndices [6 + 12*AXIS_COUNT];
    GLushort axisIndices [6];

     for (int i = 0; i < 6; i++)
     {
         axisIndices[i] = i%2 ? i/2 + 1 : 0;
     }

//     for (i = 6; i < 6 + 2*AXIS_COUNT; i += 2)
//     {
//         axisIndices[i] = ((i - 6)/2)%AXIS_COUNT + 4;
//         axisIndices[i + 1] = ((i - 6)/2)%AXIS_COUNT + AXIS_COUNT + 4;
//     }
//     for (i = 6 + 2*AXIS_COUNT; i < 6 + 4*AXIS_COUNT; i += 2)
//     {
//         axisIndices[i] = 2*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//         axisIndices[i + 1] = 3*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//     }

//     for (i = 6 + 4*AXIS_COUNT; i < 6 + 6*AXIS_COUNT; i += 2)
//     {
//         axisIndices[i] = 2*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//         axisIndices[i + 1] = 4*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//     }

//     for (i = 6 + 6*AXIS_COUNT; i < 6 + 8*AXIS_COUNT; i += 2)
//     {
//         axisIndices[i] = 5*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//         axisIndices[i + 1] = 6*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//     }
//     for (i = 6 + 8*AXIS_COUNT; i < 6 + 10*AXIS_COUNT; i += 2)
//     {
//         axisIndices[i] = 0*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//         axisIndices[i + 1] = 7*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//     }

//     for (i = 6 + 10*AXIS_COUNT; i < 6 + 12*AXIS_COUNT; i += 2)
//     {
//         axisIndices[i] = 5*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//         axisIndices[i + 1] = 8*AXIS_COUNT + ((i - 6)/2)%AXIS_COUNT + 4;
//     }

    ///                             Выставляем камеру
    matrixProjection.perspective(60.0f, (float)width()/height(), 0.1f, 100.0f);
    posEye = QVector3D(FARSIDE*zoom, FARSIDE*zoom, FARSIDE*zoom);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(0, 1, 0);
    matrixCamera.lookAt(posEye, posCenter, posUp);

    matrixWorld.setToIdentity();

    QOpenGLShader *shaderFaceVert;
    QOpenGLShader *shaderFaceFrag;
    QOpenGLShader *shaderBorderFrag;
    QOpenGLShader *shaderBorderVert;
    QOpenGLShader *shaderLineFrag;
    QOpenGLShader *shaderLineVert;
    QOpenGLShader *shaderShadowmapFrag;
    QOpenGLShader *shaderShadowmapVert;
    QOpenGLShader *shaderTextureFrag;
    QOpenGLShader *shaderTextureVert;
    QOpenGLShader *shaderDepthTextureFrag;
    QOpenGLShader *shaderDepthTextureVert;

    shaderFaceVert = new QOpenGLShader(QOpenGLShader::Vertex);
    shaderFaceVert->compileSourceFile(":/shaders/shaders/shaderFace.vsh");

    shaderFaceFrag = new QOpenGLShader(QOpenGLShader::Fragment);
    shaderFaceFrag->compileSourceFile(":/shaders/shaders/shaderFace.fsh");

    shaderBorderVert = new QOpenGLShader(QOpenGLShader::Vertex);
    shaderBorderVert->compileSourceFile(":/shaders/shaders/shaderBorder.vsh");

    shaderBorderFrag = new QOpenGLShader(QOpenGLShader::Fragment);
    shaderBorderFrag->compileSourceFile(":/shaders/shaders/shaderBorder.fsh");

    shaderLineVert = new QOpenGLShader(QOpenGLShader::Vertex);
    shaderLineVert->compileSourceFile(":/shaders/shaders/shaderLine.vsh");

    shaderLineFrag = new QOpenGLShader(QOpenGLShader::Fragment);
    shaderLineFrag->compileSourceFile(":/shaders/shaders/shaderLine.fsh");

    shaderShadowmapFrag = new QOpenGLShader(QOpenGLShader::Fragment);
    shaderShadowmapFrag->compileSourceFile(":/shaders/shaders/shaderShadow.fsh");

    shaderShadowmapVert = new QOpenGLShader(QOpenGLShader::Vertex);
    shaderShadowmapVert->compileSourceFile(":/shaders/shaders/shaderShadow.vsh");

    shaderTextureVert = new QOpenGLShader(QOpenGLShader::Vertex);
    shaderTextureVert->compileSourceFile(":/shaders/shaders/shaderTexture.vsh");

    shaderTextureFrag = new QOpenGLShader(QOpenGLShader::Fragment);
    shaderTextureFrag->compileSourceFile(":/shaders/shaders/shaderTexture.fsh");

    shaderDepthTextureVert = new QOpenGLShader(QOpenGLShader::Vertex);
    shaderDepthTextureVert->compileSourceFile(":/shaders/shaders/shaderDepthTexture.vsh");

    shaderDepthTextureFrag = new QOpenGLShader(QOpenGLShader::Fragment);
    shaderDepthTextureFrag->compileSourceFile(":/shaders/shaders/shaderDepthTexture.fsh");

    ///                         Подготавливаем программы шейдеров

    faceProgram = new QOpenGLShaderProgram(this);
    faceProgram->addShader(shaderFaceVert);
    faceProgram->addShader(shaderFaceFrag);
    faceProgram->link();
    faceProgram->bind();
    faceProgram->release();

    blackBorderProgram = new QOpenGLShaderProgram(this);
    blackBorderProgram->addShader (shaderBorderVert);
    blackBorderProgram->addShader(shaderBorderFrag);
    blackBorderProgram->link ();
    blackBorderProgram->bind ();
    blackBorderProgram->release ();

    shadowmapProgram = new QOpenGLShaderProgram(this);
    shadowmapProgram->addShader(shaderShadowmapVert);
    shadowmapProgram->addShader(shaderShadowmapFrag);
    shadowmapProgram->link();
    shadowmapProgram->bind();

    textureProgram = new QOpenGLShaderProgram(this);
    textureProgram->addShader(shaderTextureVert);
    textureProgram->addShader(shaderTextureFrag);
    textureProgram->link();
    textureProgram->bind();

    textureDepthProgram = new QOpenGLShaderProgram(this);
    textureDepthProgram->addShader(shaderTextureVert);
    textureDepthProgram->addShader(shaderTextureFrag);
    textureDepthProgram->link();
    textureDepthProgram->bind();

    lineProgram = new QOpenGLShaderProgram(this);
    lineProgram->addShader (shaderLineVert);
    lineProgram->addShader (shaderLineFrag);
    lineProgram->link ();
    lineProgram->bind ();

    axisVertBuffer.create ();
    axisVertBuffer.bind ();
    axisVertBuffer.setUsagePattern (QOpenGLBuffer::StaticDraw);
    axisVertBuffer.allocate (axisBufferData, 4*sizeof(Vertex));

    axisIndexBuffer.create ();
    axisIndexBuffer.bind ();
    axisIndexBuffer.setUsagePattern (QOpenGLBuffer::StaticDraw);
    axisIndexBuffer.allocate (axisIndices, 6*sizeof(GLushort));

    axisVertBuffer.release ();
    axisIndexBuffer.release ();

    lineProgram->release();

    delete shaderFaceVert;
    delete shaderFaceFrag;

}

void MainField::drawGrid()
{
    vao.bind();
    if (drawGridType == Arrow)
    {
        QVector<QVector3D> lineData;
        QVector<QVector3D> arrowheadData;
        for (int i = 0; i < grid.size(); i++)
        {
            lineData.append(grid[i].botCenter);
            lineData.append(grid[i].topCenter);
            arrowheadData.append(grid[i].topCenter);
            arrowheadData.append (grid[i].arrowHead);
        }

        blackBorderProgram->bind();

        QMatrix4x4 matrix = setMatrix();
        int matrixLocation = blackBorderProgram->uniformLocation ("matrixView");
        blackBorderProgram->setUniformValue(matrixLocation, matrix);

        int vertexLocation = blackBorderProgram->attributeLocation("posAttr");

        QOpenGLBuffer vertBufferLines;
        QOpenGLBuffer vertBufferArrowheads;

        vertBufferArrowheads.create();
        vertBufferArrowheads.bind();
        vertBufferArrowheads.allocate(arrowheadData.data(), arrowheadData.size()*sizeof(QVector3D));

        vertBufferLines.create();
        vertBufferLines.bind();
        vertBufferLines.allocate(lineData.data(), lineData.size()*sizeof(QVector3D));

        blackBorderProgram->enableAttributeArray(vertexLocation);
        blackBorderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
    //    lineCylinderProgram->setAttributeValue (1, QColor(Qt::green));

        glDrawArrays(GL_LINES, 0, lineData.size());

        vertBufferArrowheads.bind();
        blackBorderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        for (int i = 0; i < grid.size(); i++)
        {
            glDrawArrays(GL_TRIANGLE_FAN, i*61, 61);
        }
    } else if (drawGridType == Points)
    {
        QVector<QVector3D> lineData;
        for (int i = 0; i < grid.size(); i++)
        {
            lineData.append((grid[i].botCenter + grid[i].topCenter)/2);
        }

        blackBorderProgram->bind();

        QMatrix4x4 matrix = setMatrix();
        int matrixLocation = blackBorderProgram->uniformLocation ("matrixView");
        blackBorderProgram->setUniformValue(matrixLocation, matrix);

        int vertexLocation = blackBorderProgram->attributeLocation("posAttr");

        QOpenGLBuffer vertBufferLines;

        vertBufferLines.create();
        vertBufferLines.bind();
        vertBufferLines.allocate(lineData.data(), lineData.size()*sizeof(QVector3D));

        blackBorderProgram->enableAttributeArray(vertexLocation);
        blackBorderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
    //    lineCylinderProgram->setAttributeValue (1, QColor(Qt::green));

        glDrawArrays(GL_POINTS, 0, lineData.size());
    }
    vao.release();
}


void MainField::drawFreeVortons()
{
    vao.bind();
    QVector<QVector3D> lineData;
    QVector<QVector3D> arrowheadData;
    if (drawFreeType == Arrow)
    {
        for (int i = 0; i < freeVortons.size(); i++)
        {
            lineData.append(freeVortons[i].botCenter);
            lineData.append(freeVortons[i].topCenter);
            arrowheadData.append(freeVortons[i].topCenter);
            arrowheadData.append (freeVortons[i].arrowHead);
        }

        blackBorderProgram->bind();

        QMatrix4x4 matrix = setMatrix();
        int matrixLocation = blackBorderProgram->uniformLocation ("matrixView");
        blackBorderProgram->setUniformValue(matrixLocation, matrix);

        int vertexLocation = blackBorderProgram->attributeLocation("posAttr");

        QOpenGLBuffer vertBufferLines;
        QOpenGLBuffer vertBufferArrowheads;

        vertBufferArrowheads.create();
        vertBufferArrowheads.bind();
        vertBufferArrowheads.allocate(arrowheadData.data(), arrowheadData.size()*sizeof(QVector3D));

        vertBufferLines.create();
        vertBufferLines.bind();
        vertBufferLines.allocate(lineData.data(), lineData.size()*sizeof(QVector3D));

        blackBorderProgram->enableAttributeArray(vertexLocation);
        blackBorderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        glDrawArrays(GL_LINES, 0, lineData.size());

        vertBufferArrowheads.bind();
        blackBorderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        for (int i = 0; i < arrowheadData.size(); i++)
        {
            glDrawArrays(GL_TRIANGLE_FAN, i*61, 61);
        }

    } else if (drawFreeType == Points)
    {
        for (int i = 0; i < freeVortons.size(); i++)
        {
            lineData.append((freeVortons[i].botCenter + freeVortons[i].topCenter)/2);
        }

        blackBorderProgram->bind();

        QMatrix4x4 matrix = setMatrix();
        int matrixLocation = blackBorderProgram->uniformLocation ("matrixView");
        blackBorderProgram->setUniformValue(matrixLocation, matrix);

        int vertexLocation = blackBorderProgram->attributeLocation("posAttr");

        QOpenGLBuffer vertBufferLines;

        vertBufferLines.create();
        vertBufferLines.bind();
        vertBufferLines.allocate(lineData.data(), lineData.size()*sizeof(QVector3D));

        blackBorderProgram->enableAttributeArray(vertexLocation);
        blackBorderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        glDrawArrays(GL_POINTS, 0, lineData.size());
    }

    vao.release();
}


const int NUMBER_OF_SIDES = 60;


void MainField::drawCylinder()
{
    vao.bind();
    QVector3D topCenter(0, axisLength, 0);
    QVector3D botCenter(0, 0, 0);
    float R = radius;

    QVector4D axis(topCenter - botCenter);

    QMatrix4x4 rotation;
    QVector3D center = topCenter + botCenter;
    center /= 2;
    rotation.translate (center);
    rotation.scale (R, axis.length()/2, R);
    axis.normalize();
    rotation.rotate (2*qRadiansToDegrees(atan(axis.x())), 0, 0, 1);
    rotation.rotate (2*qRadiansToDegrees(atan2(axis.y(), 1)), 0, 1, 0);
    rotation.rotate (2*qRadiansToDegrees(atan2(axis.z(), 1)), 1, 0, 0);

    // Заполняем точки и цвета цилиндра
    Vertex vertexBufferData[2*NUMBER_OF_SIDES + 2];
    QVector3D normals [2*NUMBER_OF_SIDES + 2];
    vertexBufferData[2*NUMBER_OF_SIDES].coord = rotation*QVector4D(0.0, 1.0, 0.0, 1.0);
    vertexBufferData[2*NUMBER_OF_SIDES + 1].coord = rotation*QVector4D(0.0, -1.0, 0.0, 1.0);
    for (int i = 0; i < NUMBER_OF_SIDES; i++)
    {
        QVector4D temp;
        temp[0] = qCos(qDegreesToRadians(static_cast<float>(i)*360.0/NUMBER_OF_SIDES));
        temp[2] = qSin(qDegreesToRadians(static_cast<float>(i)*360.0/NUMBER_OF_SIDES));
        temp[1] = 1;
        temp[3] = 1;
        vertexBufferData[i].coord = rotation*temp;
        temp[1] = -1;
        vertexBufferData[i + NUMBER_OF_SIDES].coord = rotation*temp;
    }

    normals[NUMBER_OF_SIDES + 1] = QVector3D(0.0, 1.0, 0.0);
    normals[NUMBER_OF_SIDES + 2] = QVector3D(0.0, -1.0, 0.0);

    for (int i = 0; i < NUMBER_OF_SIDES; i++)
    {
        normals[i] = QVector3D::normal(vertexBufferData[i].coord.toVector3D(),
                                        vertexBufferData[(i - 1 + NUMBER_OF_SIDES)%NUMBER_OF_SIDES].coord.toVector3D(),
                                        vertexBufferData[i%NUMBER_OF_SIDES].coord.toVector3D());
        normals[i] += QVector3D::normal(vertexBufferData[i].coord.toVector3D(),
                                       vertexBufferData[i%NUMBER_OF_SIDES].coord.toVector3D(),
                                       vertexBufferData[(i + 1)%NUMBER_OF_SIDES + NUMBER_OF_SIDES].coord.toVector3D());
        normals[i] += QVector3D::normal(vertexBufferData[i].coord.toVector3D(),
                                        vertexBufferData[(i + 1)%NUMBER_OF_SIDES].coord.toVector3D(),
                                        QVector3D(0.0, 0.0, 0.0));
        normals[i] += QVector3D::normal(vertexBufferData[(i - 1 + NUMBER_OF_SIDES)%NUMBER_OF_SIDES].coord.toVector3D(),
                                        vertexBufferData[i].coord.toVector3D(),
                                        QVector3D(0.0, 0.0, 0.0));

    }

    for (int i = NUMBER_OF_SIDES; i < 2*NUMBER_OF_SIDES; i++)
    {
        normals[i] = QVector3D::normal(vertexBufferData[i].coord.toVector3D(),
                                        vertexBufferData[i%NUMBER_OF_SIDES + NUMBER_OF_SIDES].coord.toVector3D(),
                                        vertexBufferData[(i + 1)%NUMBER_OF_SIDES + NUMBER_OF_SIDES].coord.toVector3D());
        normals[i] += QVector3D::normal(vertexBufferData[i].coord.toVector3D(),
                                       vertexBufferData[(i + 1)%NUMBER_OF_SIDES].coord.toVector3D(),
                                       vertexBufferData[(i + 1)%NUMBER_OF_SIDES + NUMBER_OF_SIDES].coord.toVector3D());
        normals[i] += QVector3D::normal(vertexBufferData[i].coord.toVector3D(),
                                        vertexBufferData[(i + 1)%NUMBER_OF_SIDES].coord.toVector3D(),
                                        QVector3D(0.0, 1.0, 0.0));
        normals[i] += QVector3D::normal(vertexBufferData[(i - 1 + NUMBER_OF_SIDES)%NUMBER_OF_SIDES].coord.toVector3D(),
                                        vertexBufferData[i].coord.toVector3D(),
                                        QVector3D(0.0, 1.0, 0.0));
    }

    for (int i = 0; i < 2*NUMBER_OF_SIDES + 2; i++)
    {
        normals[i].normalize();
        vertexBufferData[i].color = QVector3D(0.5, 0.5, 0.5);
    }
    //Заполняем индексы для цилиндра
    GLushort indicesSide[2*NUMBER_OF_SIDES + 2];
    GLushort indicesTop[NUMBER_OF_SIDES + 2];
    GLushort indicesBottom[NUMBER_OF_SIDES + 2];
    for (int i = 0; i < 2*NUMBER_OF_SIDES + 2; i++)
    {
        if (i % 2 == 0)
            indicesSide[i] = (i/2)%NUMBER_OF_SIDES;
        else
            indicesSide[i] = (i/2)%NUMBER_OF_SIDES + NUMBER_OF_SIDES;
    }

    indicesTop[0] = 2*NUMBER_OF_SIDES;
    indicesBottom[0] = 2*NUMBER_OF_SIDES + 1;
    for (int i = 0; i < NUMBER_OF_SIDES + 1; i++)
    {
        indicesTop[i + 1] = i%NUMBER_OF_SIDES;
        indicesBottom[i + 1] = i%NUMBER_OF_SIDES + NUMBER_OF_SIDES;
    }

    QOpenGLBuffer vertBuffer;
    QOpenGLBuffer indexSideBuffer(QOpenGLBuffer::IndexBuffer);
    QOpenGLBuffer indexTopBuffer(QOpenGLBuffer::IndexBuffer);
    QOpenGLBuffer indexBottomBuffer(QOpenGLBuffer::IndexBuffer);
    QOpenGLBuffer normalBuffer;

    normalBuffer.create();
    normalBuffer.bind();
//    normalBuffer.setUsagePattern (QOpenGLBuffer::StaticDraw);
    normalBuffer.allocate(vertexBufferData, 2*(NUMBER_OF_SIDES + 1)*sizeof(Vertex));

    vertBuffer.create();
    vertBuffer.bind();
//    vertBuffer.setUsagePattern (QOpenGLBuffer::StaticDraw);
    vertBuffer.allocate(vertexBufferData, 2*(NUMBER_OF_SIDES + 1)*sizeof(Vertex));

    indexTopBuffer.create();
    indexTopBuffer.bind();
    indexTopBuffer.allocate(indicesTop, (NUMBER_OF_SIDES + 2)*sizeof(GLushort));

    indexBottomBuffer.create();
    indexBottomBuffer.bind();
    indexBottomBuffer.allocate(indicesBottom, (NUMBER_OF_SIDES + 2)*sizeof(GLushort));

    indexSideBuffer.create();
    indexSideBuffer.bind();
    indexSideBuffer.allocate(indicesSide, (2*NUMBER_OF_SIDES+2)*sizeof(GLushort));
//    indexBuffer.setUsagePattern (QOpenGLBuffer::StaticDraw);

    QMatrix4x4 matrix = setMatrix();

    faceProgram->bind();

    int vertexLocation = faceProgram->attributeLocation("posAttr");
    int colorLocation = faceProgram->attributeLocation("colAttr");
    int normLocation = faceProgram->attributeLocation("normAttr");

    int matrixLocation = faceProgram->uniformLocation ("matrixView");
    faceProgram->setUniformValue(matrixLocation, matrix);

    faceProgram->enableAttributeArray(vertexLocation);
    faceProgram->enableAttributeArray(colorLocation);
    faceProgram->enableAttributeArray(normLocation);

    faceProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, coord), 3, sizeof(Vertex));
    faceProgram->setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(Vertex, color), 3, sizeof(Vertex));
    faceProgram->setAttributeBuffer(normLocation, GL_FLOAT, 0, 3);

    indexBottomBuffer.bind();
    glDrawElements (GL_TRIANGLE_FAN, NUMBER_OF_SIDES + 2, GL_UNSIGNED_SHORT, 0);
    indexTopBuffer.bind();
    glDrawElements (GL_TRIANGLE_FAN, NUMBER_OF_SIDES + 2, GL_UNSIGNED_SHORT, 0);
    indexSideBuffer.bind();
    glDrawElements (GL_TRIANGLE_STRIP, 2*NUMBER_OF_SIDES+2, GL_UNSIGNED_SHORT, 0);

    faceProgram->release();

    vertBuffer.destroy();
    normalBuffer.destroy();
    indexSideBuffer.destroy();
    indexTopBuffer.destroy();
    indexBottomBuffer.destroy();
    vao.release();
}


void MainField::drawSphere(double r)
{
    vao.release();
    const int PHI_STEPS = 60;
    const int ITHA_STEPS = 60;
    const double PHI_STEP = 180.0 / PHI_STEPS;
    const double ITHA_STEP = 360.0 / ITHA_STEPS;
    const int VERTEX_COUNT = 2 + (PHI_STEPS - 2)*ITHA_STEPS;
    const int INDEX_COUNT = 2*(PHI_STEPS - 2)*(ITHA_STEPS + 1);

    Vertex vertexBufferData[VERTEX_COUNT];
    QVector3D normals[VERTEX_COUNT];

    vertexBufferData[0].coord = QVector4D(0.0, r, 0.0, 1.0);
    vertexBufferData[0].color = QVector3D(0.0, 1.0, 0.0);
    vertexBufferData[VERTEX_COUNT - 1].coord = QVector4D(0.0, -r, 0.0, 1.0);

    for (int i = 1; i < PHI_STEPS - 1; i++)
    {
        for (int j = 0; j < ITHA_STEPS; j++)
        {
            vertexBufferData[PHI_STEPS*(i - 1) + j + 1].coord = QVector4D
                    (
                        r*qSin(qDegreesToRadians(PHI_STEP*i))*qCos(qDegreesToRadians(ITHA_STEP*j)),
                        r*qCos(qDegreesToRadians(PHI_STEP*i)),
                        r*qSin(qDegreesToRadians(PHI_STEP*i))*qSin(qDegreesToRadians(ITHA_STEP*j)),
                        1.0
                        );
        }
    }

    for (int i = 0; i < VERTEX_COUNT; i++)
    {
        normals[i] = vertexBufferData[i].coord.toVector3D();
        vertexBufferData[i].color = QVector3D(0.5, 0.5, 0.5);
    }

    GLushort indicesTop[ITHA_STEPS + 2];
    GLushort indicesBottom[ITHA_STEPS + 2];
    GLushort indicesSides[INDEX_COUNT];

//    int i = 0;
    indicesTop[0] = 0;
    for (int i = 1; i < ITHA_STEPS + 2; i++)
    {
        indicesTop[i] = (i - 1)%ITHA_STEPS + 1;
    }
    indicesBottom[0] = VERTEX_COUNT - 1;
    for (int i = 1; i < ITHA_STEPS + 2; i++)
    {
        indicesBottom[i] = VERTEX_COUNT - (i - 1)%ITHA_STEPS - 2;
    }
    for (int i = 0; i < PHI_STEPS - 2; i++)
    {
        for (int j = 0; j < 2*(ITHA_STEPS + 1); j += 2)
        {
            indicesSides[j + 2*i*(ITHA_STEPS + 1)] = (j/2)%ITHA_STEPS + i*ITHA_STEPS + 1;
            indicesSides[j + 2*i*(ITHA_STEPS + 1) + 1] = (j/2)%ITHA_STEPS + (i + 1)*ITHA_STEPS + 1;
        }
    }
//    for (int i; i < VERTEX_COUNT - 1; i++)
//    {
//        indicesTop[3*i] = VERTEX_COUNT - 1;
//        indicesTop[3*i + 1] = i%ITHA_STEPS + VERTEX_COUNT - ITHA_STEPS;
//        indicesTop[3*i + 2] = (i + 1)%ITHA_STEPS + VERTEX_COUNT - ITHA_STEPS;
//    }

    QOpenGLBuffer vertBuffer;
    QOpenGLBuffer normBuffer;
    QOpenGLBuffer indexBufferTop(QOpenGLBuffer::IndexBuffer);
    QOpenGLBuffer indexBufferBottom(QOpenGLBuffer::IndexBuffer);
    QOpenGLBuffer indexBufferSides(QOpenGLBuffer::IndexBuffer);

    normBuffer.create();
    normBuffer.bind();
    normBuffer.allocate(normals, VERTEX_COUNT*sizeof(QVector3D));

    vertBuffer.create();
    vertBuffer.bind();
//    vertBuffer.setUsagePattern (QOpenGLBuffer::StaticDraw);
    vertBuffer.allocate(vertexBufferData, VERTEX_COUNT*sizeof(Vertex));

    indexBufferTop.create();
    indexBufferTop.bind();
    indexBufferTop.allocate(indicesTop, (ITHA_STEPS + 2)*sizeof(GLushort));

    indexBufferBottom.create();
    indexBufferBottom.bind();
    indexBufferBottom.allocate(indicesBottom, (ITHA_STEPS + 2)*sizeof(GLushort));

    indexBufferSides.create();
    indexBufferSides.bind();
    indexBufferSides.allocate(indicesSides, INDEX_COUNT*sizeof(GLushort));

    QMatrix4x4 depthMatrix;
//    depthMatrix.ortho(-4, 4, -4, 4, 4, -4);
    depthMatrix.perspective (60.0f, static_cast<float>(width())/static_cast<float>(height()), 0.1f, 100.0f);
    depthMatrix.lookAt(QVector3D(0, 4, 0), QVector3D(0, 0, 0), QVector3D(1, 0, 0));

//    lightTexture = new QOpenGLTexture(QOpenGLTexture::Target2D);
//    lightTexture->setSize(1024, 1024);
//    lightTexture->allocateStorage(QOpenGLTexture::Depth, QOpenGLTexture::Float32);
//    lightTexture->setMinMagFilters(QOpenGLTexture::Nearest, QOpenGLTexture::Nearest);
//    lightTexture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::ClampToEdge);
//    lightTexture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::ClampToEdge);
//    lightTexture->setDepthStencilMode(QOpenGLTexture::DepthMode);

    vao.bind();
    QOpenGLFramebufferObject fbo(1024, 1024);
//    fbo.setAttachment (QOpenGLFramebufferObject::Depth);
//    fbo.addColorAttachment (1024, 1024, 0);
//    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, lightTexture->textureId(), 0);

    shadowmapProgram->bind();
//    QMatrix4x4 matrix = setMatrix();

    int matrixLocation = shadowmapProgram->uniformLocation ("matrixView");
    shadowmapProgram->setUniformValue(matrixLocation, depthMatrix);

    int vertexLocation = shadowmapProgram->attributeLocation("posAttr");
    shadowmapProgram->enableAttributeArray(vertexLocation);

    shadowmapProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, coord), 3, sizeof(Vertex));

    fbo.bind();

    indexBufferTop.bind();
    glDrawElements(GL_TRIANGLE_FAN, ITHA_STEPS + 2, GL_UNSIGNED_SHORT, 0);

    indexBufferSides.bind();
    glDrawElements(GL_TRIANGLE_STRIP, INDEX_COUNT, GL_UNSIGNED_SHORT, 0);

    indexBufferBottom.bind();
    glDrawElements(GL_TRIANGLE_FAN, ITHA_STEPS + 2, GL_UNSIGNED_SHORT, 0);

//    QImage image = fbo.toImage();
//    sendScreenshot(image);
    GLuint tex = fbo.takeTexture();

    faceProgram->bind();

    /*int */vertexLocation = faceProgram->attributeLocation("posAttr");
    int colorLocation = faceProgram->attributeLocation("colAttr");
    int normalLocation = faceProgram->attributeLocation("normAttr");

    faceProgram->enableAttributeArray(vertexLocation);
    faceProgram->enableAttributeArray(colorLocation);
    faceProgram->enableAttributeArray(normalLocation);

    QMatrix4x4 matrix = setMatrix();
    /*int */matrixLocation = faceProgram->uniformLocation ("matrixView");
    faceProgram->setUniformValue(matrixLocation, matrix);
    matrixLocation = faceProgram->uniformLocation("matrixDepth");
    faceProgram->setUniformValue(matrixLocation, biasMatrix * depthMatrix);
    int texLocation = faceProgram->uniformLocation("shadowMap");
    faceProgram->setUniformValue(texLocation, tex);

    faceProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, coord), 3, sizeof(Vertex));
    faceProgram->setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(Vertex, color), 3, sizeof(Vertex));
    faceProgram->setAttributeBuffer(normalLocation, GL_FLOAT, 0, 3);

    indexBufferTop.bind();
    glDrawElements(GL_TRIANGLE_FAN, ITHA_STEPS + 2, GL_UNSIGNED_SHORT, 0);

    indexBufferSides.bind();
    glDrawElements(GL_TRIANGLE_STRIP, INDEX_COUNT, GL_UNSIGNED_SHORT, 0);

    indexBufferBottom.bind();
    glDrawElements(GL_TRIANGLE_FAN, ITHA_STEPS + 2, GL_UNSIGNED_SHORT, 0);

    vertBuffer.destroy();
    indexBufferTop.destroy();
    indexBufferSides.destroy();
    indexBufferBottom.destroy();
    vao.release();
}


void MainField::drawAxis()
{
    vao.bind();
    axisVertBuffer.bind ();
    axisIndexBuffer.bind();
    lineProgram->bind();


    int matrixLocation = lineProgram->uniformLocation ("matrixView");
    QMatrix4x4 matrix = setMatrix ();
    lineProgram->setUniformValue(matrixLocation, matrix);

    int vertexLocation = lineProgram->attributeLocation("posAttr");
    int colorLocation = lineProgram->attributeLocation("colAttr");

    lineProgram->enableAttributeArray(vertexLocation);
    lineProgram->enableAttributeArray(colorLocation);

    lineProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, coord), 3, sizeof(Vertex));
    lineProgram->setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(Vertex, color), 3, sizeof(Vertex));

    glDrawElements (GL_LINES, 6, GL_UNSIGNED_SHORT, 0);

    lineProgram->release();
    axisIndexBuffer.release();
    axisVertBuffer.release ();

    glLineWidth (1.0);
    vao.release();
}

void MainField::drawMarker()
{
    vao.bind();
    lineProgram->bind();
    int matrixLocation = lineProgram->uniformLocation ("matrixView");
    QMatrix4x4 matrix = setMatrix();
    lineProgram->setUniformValue(matrixLocation, matrix);

    int vertexLocation = lineProgram->attributeLocation("posAttr");
    int colorLocation = lineProgram->attributeLocation("colAttr");


    QOpenGLBuffer vertBuffer;

    vertBuffer.create();
    vertBuffer.bind();
    vertBuffer.allocate(marker, 4*sizeof(Vertex));

    lineProgram->enableAttributeArray(vertexLocation);
    lineProgram->enableAttributeArray(colorLocation);

    lineProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, coord), 3, sizeof(Vertex));
    lineProgram->setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(Vertex, color), 3, sizeof(Vertex));

    glLineWidth(1.0);
    glDrawArrays(GL_LINES, 0, 4);

    vertBuffer.destroy();
    lineProgram->release();
    vao.release();
}

void MainField::updateMarker()
{
    marker[0].coord = QVector4D(0.0, 1.0, 0.0, 0.0);
    marker[1].coord = QVector4D(0.0, -1.0, 0.0, 0.0);
    marker[2].coord = QVector4D(1.0, 0.0, 0.0, 0.0);
    marker[3].coord = QVector4D(-1.0, 0.0, 0.0, 0.0);
    for (int i = 0; i < 4; i++)
    {

        marker[i].coord = setMatrix().inverted() * marker[i].coord;
        marker[i].coord += posCenter;
    }

}

void MainField::resizeGL(int w, int h)
{
//    w = (w < h) ? w : h;
//    h = (w < h) ? w : h;

    glViewport (0, 0, w, h);

}


void MainField::paintGL()
{
    glViewport (0, 0, width(), height());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(backgroundColor.redF(), backgroundColor.greenF(), backgroundColor.blueF(), 1.0);
//    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable (GL_DEPTH_TEST);
    glDepthFunc (GL_LESS);

    switch (Shape) {
    case MainField::Cylinder: drawCylinder(); break;
    case MainField::Sphere: drawSphere(radius); break;
    default: break;
    }
    drawGrid();
    drawFreeVortons();
    drawAxis();
    if (moving || turning)
    {
        drawMarker();
    }
//    drawSphere(1.0);
//    drawAxis();
}

const int TURN_BUTTON = Qt::MiddleButton;
const int MOVE_BUTTON = Qt::LeftButton;

void MainField::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == TURN_BUTTON)
    {
        turning = true;
        mousePositionStart = e->pos();
        updateMarker();
    }
    if (e->button() == MOVE_BUTTON)
    {
        moving = true;
        mousePositionStart = e->pos();
        updateMarker();
    }
}

const float TURNING_ANGLE = 90.0f;
const float MOVE_ACCELERATION = 2.0f;

void MainField::mouseMoveEvent(QMouseEvent* e)
{
    if (turning)
    {
        if (this->rect().contains (e->pos()))
        {
            QVector4D vectorUp(0.0f, 1.0f, 0.0f, 0.0f);
            vectorUp = (setMatrix().inverted())*(vectorUp);
            matrixRotation.setToIdentity();
            int rotationX = static_cast <int> (TURNING_ANGLE*(e->x() - mousePositionStart.x())/width());
            qNormalizeAngle(rotationX);

            matrixRotation.translate(posCenter);
            matrixRotation.rotate(rotationX, vectorUp.toVector3D());
            matrixRotation.translate(-posCenter);
            QVector4D vectorLeft(1.0f, 0.0f, 0.0f, 0.0f);
            vectorLeft = (setMatrix().inverted())*(vectorLeft + QVector4D(posCenter, 0.0));
            int rotationY = static_cast <int> (TURNING_ANGLE*(e->y() - mousePositionStart.y())/height());
            qNormalizeAngle(rotationY);

            matrixRotation.translate(posCenter);
            matrixRotation.rotate(-rotationY, vectorLeft.toVector3D());
            matrixRotation.translate(-posCenter);

            //        QCursor cursor = this->cursor();
            //        cursor.setPos(mapToGlobal(QPoint(this->width()/2, this->height()/2)));
            //        this->setCursor(cursor);
            update();
        }
    } else if (moving)
    {
        if (this->rect().contains (e->pos()))
        {
            matrixTranslation.setToIdentity();
            posMoved = QVector3D(MOVE_ACCELERATION*zoom*static_cast<float>(e->pos().x() - mousePositionStart.x())/width(),
                           -MOVE_ACCELERATION*zoom*static_cast<float>(e->pos().y() - mousePositionStart.y())/height(),
                           0);
            matrixTranslation.translate(posMoved);

            update();
        }
    }
}

void MainField::mouseReleaseEvent(QMouseEvent* e)
{
    if (turning && e->button() == TURN_BUTTON)
    {
        turning = false;
        matrixWorld = matrixRotation*matrixWorld;
        matrixRotation.setToIdentity();
//        QCursor cursor = this->cursor();
//        cursor.setPos(mapToGlobal(mousePositionStart));
//        cursor.setShape (Qt::ArrowCursor);
//        this->setCursor(cursor);
    }
    if (moving && e->button() == MOVE_BUTTON)
    {
        moving = false;
        QVector4D trans(0, 0, 0, 1);
//        posCenter = -QVector4D(matrixTranslation*trans).toVector3D();
        posCenter -= posMoved;
        posEye -= posMoved;
        QString str = QString().number((posCenter - posEye).length());
        emit testStuff(str);
        matrixCamera.setToIdentity();
        matrixTranslation.setToIdentity();
        matrixCamera.lookAt(posEye, posCenter, posUp);
    }
    update();
}

void MainField::setPlaneXY()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(0, 0, FARSIDE*zoom);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(0, 1, 0);
    matrixCamera.lookAt(posEye, posCenter, posUp);
//    matrixCamera.lookAt (QVector3D(), QVector3D(0, 0, 0), QVector3D());
    matrixWorld.setToIdentity();
    update();
}

void MainField::setPlaneYX()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(0, 0, -FARSIDE*zoom);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(1, 0, 0);
    matrixCamera.lookAt(posEye, posCenter, posUp);
//    matrixCamera.lookAt (QVector3D(), QVector3D(0, 0, 0), QVector3D());
    matrixWorld.setToIdentity();
    update();
}

void MainField::setPlaneXZ()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(0, -FARSIDE*zoom, 0);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(0, 0, 1);
    matrixCamera.lookAt(posEye, posCenter, posUp);
//    matrixCamera.lookAt (QVector3D(), QVector3D(0, 0, 0), QVector3D());
    matrixWorld.setToIdentity();
    update();
}

void MainField::setPlaneZX()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(0, FARSIDE*zoom, 0);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(1, 0, 0);
    matrixCamera.lookAt(posEye, posCenter, posUp);
//    matrixCamera.lookAt (QVector3D(), QVector3D(0, 0, 0), QVector3D());
    matrixWorld.setToIdentity();
    update();
}

void MainField::setPlaneYZ()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(FARSIDE*zoom, 0, 0);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(0, 0, 1);
    matrixCamera.lookAt(posEye, posCenter, posUp);
    matrixWorld.setToIdentity();
    update();
}

void MainField::setPlaneZY()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(-FARSIDE*zoom, 0, 0);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(0, 1, 0);
    matrixCamera.lookAt(posEye, posCenter, posUp);
    matrixWorld.setToIdentity();
    update();
}

void MainField::zoomIn()
{
    if (zoom > 0.1)
        setZoom(zoom - 0.1);
    emit zoomChanged(zoom * 100.0);
    update();
}

void MainField::zoomOut()
{
    if (zoom < 2.5)
        setZoom(zoom + 0.1);
    else
        setZoom(2.5);
    emit zoomChanged(zoom * 100.0);
    update();
}

void MainField::updateZoom(double zoom)
{
    setZoom(zoom/100.0);
}

void MainField::resetPlane()
{
    matrixCamera = QMatrix4x4();
    posEye = QVector3D(FARSIDE*zoom, FARSIDE*zoom, FARSIDE*zoom);
    posCenter = QVector3D(0, 0, 0);
    posUp = QVector3D(0, 1, 0);
    matrixCamera.lookAt(posEye, posCenter, posUp);
//    matrixCamera.lookAt(QVector3D(), QVector3D(0, 0, 0), QVector3D());
    setZoom(1.0);
    emit zoomChanged(zoom * 100.0);
    matrixWorld.setToIdentity();
    update();
}

void MainField::addVorton (QVector3D botCenter, QVector3D topCenter, SArrow::vort_type drawType)
{
    const double MINIMAL_LENGTH = 0.05;
    SArrow cylinder;
    cylinder.topCenter = topCenter;
    cylinder.botCenter = botCenter;
    cylinder.vortonType = drawType;
    if (drawType == SArrow::Grid)
    {
        QMatrix4x4 translation;
        QVector4D axis(botCenter - topCenter, 1.0f);
        double r = axis.toVector3D().length();
//        translation.scale(0.05*axis.length(), 0.1*axis.length(), 0.05*axis.length());

        if (r > MINIMAL_LENGTH)
            translation.translate(topCenter + 0.1*axis.toVector3D()/*.normalized()*/);
        else
            translation.translate(topCenter + 0.3*axis.toVector3D()/*.normalized()*/);

        axis.normalize();
        translation.rotate (2*qRadiansToDegrees(atan(axis.x())), 0, 0, 1);
        translation.rotate (2*qRadiansToDegrees(atan2(axis.y(), 1)), 0, 1, 0);
        translation.rotate (2*qRadiansToDegrees(atan2(axis.z(), 1)), 1, 0, 0);
        QVector4D temp;
        for (int i = 0; i < 60; i++)
        {
            if (r > MINIMAL_LENGTH)
                temp = QVector4D(0.02*r*qSin(qDegreesToRadians(i*6.0f)),
                             0.0f,
                             0.02*r*qCos(qDegreesToRadians(i*6.0f)),
                             1.0f
                             );
            else
                temp = QVector4D(0.15*r*qSin(qDegreesToRadians(i*6.0f)),
                             0.0f,
                             0.15*r*qCos(qDegreesToRadians(i*6.0f)),
                             1.0f
                             );
            temp = translation*temp;
            cylinder.arrowHead.append(temp.toVector3D());
        }
        grid.append(cylinder);
    } else if (drawType == SArrow::Free)
    {
        QMatrix4x4 translation;
        QVector4D axis(botCenter - topCenter, 1.0f);
        cylinder.middle = (botCenter + topCenter) / 2;
        double r = axis.toVector3D().length();

        if (r > MINIMAL_LENGTH)
            translation.translate(topCenter + 0.1*axis.toVector3D());
        else
            translation.translate(topCenter + 0.3*axis.toVector3D());

        axis.normalize();
        translation.rotate (2*qRadiansToDegrees(atan(axis.x())), 0, 0, 1);
        translation.rotate (2*qRadiansToDegrees(atan2(axis.y(), 1)), 0, 1, 0);
        translation.rotate (2*qRadiansToDegrees(atan2(axis.z(), 1)), 1, 0, 0);
        QVector4D temp;
        for (int i = 0; i < 60; i++)
        {
            if (r > MINIMAL_LENGTH)
                temp = QVector4D(0.02*r*qSin(qDegreesToRadians(i*6.0f)),
                             0.0f,
                             0.02*r*qCos(qDegreesToRadians(i*6.0f)),
                             1.0f
                             );
            else
                temp = QVector4D(0.15*r*qSin(qDegreesToRadians(i*6.0f)),
                             0.0f,
                             0.15*r*qCos(qDegreesToRadians(i*6.0f)),
                             1.0f
                             );
            temp = translation*temp;
            cylinder.arrowHead.append(temp.toVector3D());
        }
        freeVortons.append(cylinder);
    }
    update();
}

void MainField::addVorton(QVector3D botCenter, QVector3D topCenter)
{
    addVorton (botCenter, topCenter, SArrow::Free);
}

void MainField::clearCylinders(int n)
{
    if (n < 0)
    {
        grid.clear();
        freeVortons.clear();
    } else {
        grid.remove (grid.size() - n, n);
    }
    update();
}

void MainField::updateColors (QColor backColor)
{
    backgroundColor = backColor;
    update();
}

void MainField::updateFreeVortonsMode(bool asPoints)
{
    if (asPoints)
        drawFreeType = Points;
    else
        drawFreeType = Arrow;
    update();
}

void MainField::updateGridVortonsMode(bool asPoints)
{
    if (asPoints)
        drawGridType = Points;
    else
        drawGridType = Arrow;
    update();
}

void MainField::changeModel(shape newShape)
{
    Shape = newShape;
    update();
}

void MainField::setHeight (double height)
{
    this->axisLength = height;
}

void MainField::setRadius (double radius)
{
    this->radius = radius;
    update();
}
