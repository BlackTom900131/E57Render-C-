#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <thread>
#include <chrono>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <E57Format/E57Format.h> // For reading E57 files
#include <E57Format/E57Export.h>
#include <E57Format/E57SimpleReader.h>
#include <E57Format/E57SimpleData.h>
#include <E57Format/E57SimpleWriter.h>
#include <Windows.h>
#include "newtask.h"
#include "CProcPoints.h"
#include "CSortPointCloud.h"
#include "CDetector.h"

std::vector<Point> points;
std::vector<Point> pointsc;

// Class definitions
const std::vector<std::string> classNames = {
    "Floor", "Ceiling", "Walls", "Furniture", "Beams", "Columns"
};
const int classCount = 6;

// Color map for classes
const float classColors[classCount][3] = {
    {0.58f, 0.29f, 0.0f},     // Floor    -     Brownish
    {1.0f, 1.0f, 0.0f},       // Ceiling  -     Yellow
    {1.0f, 0.0f, 0.0f},       // Walls    -     Red
    {0.0f, 1.0f, 0.0f},       // Furniture-     Green
    {0.0f, 0.0f, 1.0f},       // Beams    -     Blue
    {1.0f, 0.65f, 0.0f}       // Columns  -     Orange
};

// Visibility toggles
bool showClass[classCount] = { true, true, true, true, true, true };

// Point cloud data
//std::vector<Point> points;

bool loadE571(const std::string& filename, std::vector<Point>& points)
{
    try
    {
        e57::ReaderOptions options;
        e57::Reader reader(filename, options);

        const int64_t scanCount = reader.GetData3DCount();

        if (scanCount == 0)
        {
            std::cerr << "No 3D scans found in file.\n";
            return false;
        }

        // Read first scan
        e57::Data3D data3DHeader;

        reader.ReadData3D(0, data3DHeader);
        std::cout << "Has cartesianX: " << data3DHeader.pointFields.cartesianXField << std::endl;
        std::cout << "Has cartesianY: " << data3DHeader.pointFields.cartesianYField << std::endl;
        std::cout << "Has cartesianZ: " << data3DHeader.pointFields.cartesianZField << std::endl;
        std::cout << "Has intensity: " << data3DHeader.pointFields.intensityField << std::endl;
        std::cout << "Has colorRed: " << data3DHeader.pointFields.colorRedField << std::endl;

        int64_t nColumn = 0;
        int64_t nRow = 0;

        int64_t nPointsSize = 0; // Number of points

        int64_t nGroupsSize = 0;   // Number of groups
        int64_t nCountSize = 0;    // Number of points per group
        bool bColumnIndex = false; // indicates that idElementName is "columnIndex"

        reader.GetData3DSizes(0, nRow, nColumn, nPointsSize, nGroupsSize, nCountSize, bColumnIndex);

        int64_t nSize = nRow;
        if (nSize == 0)
            nSize = 1024; // choose a chunk size
        //  Setup buffers

        int8_t* isInvalidData = NULL;
        if (data3DHeader.pointFields.cartesianInvalidStateField)
            isInvalidData = new int8_t[nSize];

        //  Setup Points Buffers
        double* xData = NULL;
        if (data3DHeader.pointFields.cartesianXField)
            xData = new double[nSize];

        double* yData = NULL;
        if (data3DHeader.pointFields.cartesianYField)
            yData = new double[nSize];

        double* zData = NULL;
        if (data3DHeader.pointFields.cartesianZField)
            zData = new double[nSize];
        //  Setup intensity buffers if present

        double* intData = NULL;
        bool bIntensity = false;
        double intRange = 0;
        double intOffset = 0;

        if (data3DHeader.pointFields.intensityField)
        {
            bIntensity = true;
            intData = new double[nSize];
            intRange = data3DHeader.intensityLimits.intensityMaximum - data3DHeader.intensityLimits.intensityMinimum;
            intOffset = data3DHeader.intensityLimits.intensityMinimum;
        }

        //  Setup color buffers if present

        uint16_t* redData = NULL;
        uint16_t* greenData = NULL;
        uint16_t* blueData = NULL;
        bool bColor = false;
        int32_t colorRedRange = 1;
        int32_t colorRedOffset = 0;
        int32_t colorGreenRange = 1;
        int32_t colorGreenOffset = 0;
        int32_t colorBlueRange = 1;
        int32_t colorBlueOffset = 0;

        if (data3DHeader.pointFields.colorRedField)
        {
            bColor = true;
            redData = new uint16_t[nSize];
            greenData = new uint16_t[nSize];
            blueData = new uint16_t[nSize];
            colorRedRange = data3DHeader.colorLimits.colorRedMaximum - data3DHeader.colorLimits.colorRedMinimum;
            colorRedOffset = data3DHeader.colorLimits.colorRedMinimum;
            colorGreenRange = data3DHeader.colorLimits.colorGreenMaximum - data3DHeader.colorLimits.colorGreenMinimum;
            colorGreenOffset = data3DHeader.colorLimits.colorGreenMinimum;
            colorBlueRange = data3DHeader.colorLimits.colorBlueMaximum - data3DHeader.colorLimits.colorBlueMinimum;
            colorBlueOffset = data3DHeader.colorLimits.colorBlueMinimum;
        }

        //  Setup the GroupByLine buffers information

        int64_t* idElementValue = NULL;
        int64_t* startPointIndex = NULL;
        int64_t* pointCount = NULL;
        if (nGroupsSize > 0)
        {
            idElementValue = new int64_t[nGroupsSize];
            startPointIndex = new int64_t[nGroupsSize];
            pointCount = new int64_t[nGroupsSize];

            if (!reader.ReadData3DGroupsData(0, nGroupsSize, idElementValue,
                startPointIndex, pointCount))
                nGroupsSize = 0;
        }
        //  Setup row / column index information

        int32_t* rowIndex = NULL;
        int32_t* columnIndex = NULL;
        if (data3DHeader.pointFields.rowIndexField)
            rowIndex = new int32_t[nSize];
        if (data3DHeader.pointFields.columnIndexField)
            columnIndex = new int32_t[nRow];
        //  Get dataReader object
        int scanindex = 0;

        // struct Data3DPointsDouble {
        //     double* cartesianX = nullptr;
        //     double* cartesianY = nullptr;
        //     double* cartesianZ = nullptr;
        //     // Other optional fields
        //     double* intensity = nullptr;
        //     bool* isInvalid = nullptr;
        //     uint16_t* rowIndex = nullptr;
        //     uint16_t* columnIndex = nullptr;
        //     // etc.
        // };

        e57::Data3DPointsDouble buffers;
        buffers.cartesianX = new double[nSize];
        buffers.cartesianY = new double[nSize];
        buffers.cartesianZ = new double[nSize];

        /*buffers.rowIndex = rowIndex;
        buffers.columnIndex = columnIndex;*/
        // Fill other fields if needed, like buffers.intensity, buffers.isInvalid, etc.
        {
            std::vector<double> xData(4096);
            std::vector<double> yData(4096);
            std::vector<double> zData(4096);

            e57::Data3DPointsDouble buffers;
            buffers.cartesianX = xData.data();
            buffers.cartesianY = yData.data();
            buffers.cartesianZ = zData.data();
            e57::CompressedVectorReader dataReader = reader.SetUpData3DPointsData(
                scanindex,
                nRow,
                buffers);
            int64_t count = 0;
            unsigned long size = 0;
            int col = 0;
            int row = 0;

            while ((size = dataReader.read()) > 0) // Each call to dataReader.read() will retrieve the next column of data.
            {
                for (long i = 0; i < size; i++)
                {
                    Point p;
                    if (columnIndex)
                        col = columnIndex[i];
                    else
                        col = 0; // point cloud case

                    if (rowIndex)
                        row = rowIndex[i];
                    else
                        row = count; // point cloud case

                    if (isInvalidData != NULL)
                        // pScan->SetPoint(row, col, xData[i], yData[i], zData[i]);
                        p.x = xData[i];

                    if (bIntensity)
                    { // Normalize intensity to 0 - 1.
                        double intensity = (intData[i] - intOffset) / intRange;
                        // pScan->SetIntensity(row, col, intensity);
                    }

                    if (bColor)
                    { // Normalize color to 0 - 255
                        int red = ((redData[i] - colorRedOffset) * 255) / colorRedRange;
                        int green = ((greenData[i] - colorGreenOffset) * 255) / colorBlueRange;
                        int blue = ((blueData[i] - colorBlueOffset) * 255) / colorBlueRange;
                        // pScan->SetColor(row, col, red, green, blue);

                        count++;
                    }
                }
            }
        }
        if (buffers.cartesianX) delete[] buffers.cartesianX;
        if (buffers.cartesianY) delete[] buffers.cartesianY;
        if (buffers.cartesianZ) delete[] buffers.cartesianZ;

        if (intData) delete[] intData;

        if (redData) delete[] redData;
        if (greenData) delete[] greenData;
        if (blueData) delete[] blueData;

        if (isInvalidData) delete[] isInvalidData;

        if (xData) delete[] xData;
        if (yData) delete[] yData;
        if (zData) delete[] zData;

        if (idElementValue) delete[] idElementValue;
        if (startPointIndex) delete[] startPointIndex;
        if (pointCount) delete[] pointCount;

        if (rowIndex) delete[] rowIndex;
        if (columnIndex) delete[] columnIndex;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error reading E57: " << e.what() << std::endl;
        return false;
    }
    return true;
}
bool loadE57(const std::string& filename, std::vector<Point>& points)
{
    e57::ReaderOptions options;
    e57::Reader reader(filename, options);
    int scanIndex = 0;

    e57::Data3D scanHeader;
    reader.ReadData3D(scanIndex, scanHeader);
    std::cout << "Has cartesianX: " << scanHeader.pointFields.cartesianXField << std::endl;
    std::cout << "Has cartesianY: " << scanHeader.pointFields.cartesianYField << std::endl;
    std::cout << "Has cartesianZ: " << scanHeader.pointFields.cartesianZField << std::endl;

    std::cout << "Has intensity: " << scanHeader.pointFields.intensityField << std::endl;

    std::cout << "Has sphericalRange: " << scanHeader.pointFields.sphericalRangeField << std::endl;
    std::cout << "Has sphericalAzimuth: " << scanHeader.pointFields.sphericalAzimuthField << std::endl;
    std::cout << "Has sphericalElevation: " << scanHeader.pointFields.sphericalElevationField << std::endl;

    std::cout << "Has colorRed: " << scanHeader.pointFields.colorRedField << std::endl;
    std::cout << "Has colorRed: " << scanHeader.pointFields.colorGreenField << std::endl;
    std::cout << "Has colorRed: " << scanHeader.pointFields.colorBlueField << std::endl;

    int64_t nPoints = scanHeader.pointCount;

    // Allocate buffers for all required fields
    std::vector<double> rangeData(nPoints);
    std::vector<double> azimuthData(nPoints);
    std::vector<double> elevationData(nPoints);

    e57::Data3DPointsDouble buffers{};
    buffers.sphericalRange = rangeData.data();
    buffers.sphericalAzimuth = azimuthData.data();
    buffers.sphericalElevation = elevationData.data();

    // If your E57 file has intensity or color, allocate and assign those too:
    // std::vector<double> intensityData(nPoints);
    // buffers.intensity = intensityData.data();

    // Set up the data reader
    e57::CompressedVectorReader dataReader = reader.SetUpData3DPointsData(
        scanIndex,
        nPoints,
        buffers);

    // Read all points
    size_t totalRead = 0;
    while (totalRead < nPoints) {
        size_t batchSize = dataReader.read();

        for (size_t i = 0; i < batchSize; ++i) {
            double r = rangeData[i];
            double az = azimuthData[i];     // in radians
            double el = elevationData[i];   // in radians

            // Convert spherical to cartesian
            double x = r * cos(el) * cos(az);
            double y = r * cos(el) * sin(az);
            double z = r * sin(el);

            Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.label = -1;

            points.push_back(p);
        }
        totalRead += batchSize;
    }
    return true;
}

// Dummy segmentation function (simulate calling Python or ML model)
void segmentPoints(std::vector<Point>& pts)
{
    // For demo, assign random classes
    for (auto& p : pts)
    {
        p.label = -1;
    }
}

GLFWwindow* initGL()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return nullptr;
    }

    // Set to OpenGL 2.1 for full legacy support
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    // Don't request core profile
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); ← remove

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Point Cloud Segmentation", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << "\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return nullptr;
    }

    glfwSwapInterval(1); // Enable vsync

    std::cout << "Renderer: " << glGetString(GL_RENDERER) << "\n";
    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << "\n";

    return window;
}

// Setup ImGui
void setupImGui(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

// Called once per frame before drawing anything
void myPerspective(float fovY, float aspect, float nearZ, float farZ)
{
    float f = 1.0f / tanf(fovY * 0.5f * 3.14159265f / 180.0f);
    float nf = 1.0f / (nearZ - farZ);

    float m[16] = { 0 };

    m[0] = f / aspect;
    m[5] = f;
    m[10] = (farZ + nearZ) * nf;
    m[11] = -1.0f;
    m[14] = (2.0f * farZ * nearZ) * nf;

    glLoadMatrixf(m);
}

void setupCamera(int windowWidth, int windowHeight)
{
    float aspect = (float)windowWidth / (float)windowHeight;

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    myPerspective(30.0, aspect, 0.1, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Apply camera transform
    glTranslatef(0.f, 0.f, -50.f);
    glRotatef(20.f, 1.f, 0.f, 0.f);
    glRotatef((float)glfwGetTime() * 10.f, 0.f, 1.f, 0.f);
}

// Render point cloud
void renderPointCloud()
{
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (const auto& p : pointsc)
    {
        // Optional: Color by label/class
        if (p.label == 1)
            glColor3f(1.0f, 0.0f, 0.0f);  // Red
        else if (p.label == 2)
            glColor3f(0.0f, 1.0f, 0.0f);  // Green
        else if (p.label == 3)
            glColor3f(0.0f, 0.0f, 1.0f);  // Blue
        else if (p.label == 4)
            glColor3f(0.0f, 0.5f, 0.5f);  // Blue
        else if (p.label == 5)
            glColor3f(0.0f, 0.5f, 1.0f);  // Blue
        else if (p.label == 6)
            glColor3f(0.0f, 1.0f, 0.5f);  // Blue
        else if (p.label == 7)
            glColor3f(0.0f, 1.0f, 1.0f);  // Blue

        glVertex3f((float)p.x, (float)p.y, (float)p.z);
    }
    glEnd();
}

void drawLine(float x1, float y1, float z1,
    float x2, float y2, float z2)
{
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 1.0f); // White

    glBegin(GL_LINES);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glEnd();
}

// Function to get the first argument (excluding the program name)
std::string GetFirstArgument()
{
    int argc = 0;
    LPWSTR* argvW = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (argvW == nullptr || argc < 2)
    {
        if (argvW)
            LocalFree(argvW);
        return "";
    }

    // Convert wide string to multi-byte string
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, argvW[1], -1, NULL, 0, NULL, NULL);
    std::string argument;
    if (size_needed > 0)
    {
        char* buffer = new char[size_needed];
        WideCharToMultiByte(CP_UTF8, 0, argvW[1], -1, buffer, size_needed, NULL, NULL);
        argument = buffer;
        delete[] buffer;
    }

    LocalFree(argvW);
    return argument;
}

// WinMain function (Win32 entry point)
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    std::string filename = "C_001.e57";
    CProcPoints procPoint;
    // std::string filename = GetFirstArgument();
    // if (filename.empty()) {
    //     //MessageBoxA(NULL, "Usage: program.exe <e57_filename>", "Error", MB_OK | MB_ICONERROR);
    //     return -1;
    // }

    // Your program logic here
    // For example:
    // MessageBoxA(NULL, ("Loading file: " + filename).c_str(), "Info", MB_OK);

    // Load point cloud
    std::cout << "Loading E57 file...\n";
    if (!loadE57(filename, points))
    {
        std::cerr << "Failed to load E57\n";
        // MessageBoxA(NULL, ("Failed to load E57 : " + filename).c_str(), "Info", MB_OK);
        return -1;
    }
    std::cout << "Loaded " << points.size() << " points.\n";

    //ProcPoints
    CSortPointCloud sortManager;
    sortManager.m_src_points = points;
    sortManager.sortPoints(0.01, 0.01, 0.01);
    sortManager.removeSmallFragment(3, 0.04);
    points = std::move(sortManager.m_dst_points);
    
    int counter = 0;
    CDetector detector;
    auto floors = detector.detectFloorAll(points,0.1,80000);
    auto wallsx = detector.detectWallX(points, 0.1, 10000);
    auto wallsy = detector.detectWallY(points, 0.1, 7000);

    if (!pointsc.empty())
        pointsc.clear();
    //for (const auto& wall : wallsx) {
    //    pointsc.insert(pointsc.end(), wall.begin(), wall.end());
    //    //counter++;
    //}
    /*for (const auto& wall : wallsy) {
        pointsc.insert(pointsc.end(), wall.begin(), wall.end());
        counter++;
    }*/
    for (const auto& floor : floors) {
        pointsc.insert(pointsc.end(), floor.begin(), floor.end());
        counter++;
    }
    
    counter = 0;
    /*procPoint.convertPoints();
    procPoint.convertPointCloud();
    procPoint.segmentPlanes();*/

    // Initialize GLFW (required for window creation)
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    // ... (other GLFW initialization, such as window creation) ...

    // ... (OpenGL initialization code) ...

    glfwSwapInterval(1);

    // Initialize OpenGL & ImGui
    GLFWwindow* window = initGL();
    if (!window)
        return -1;

    // ... (ImGui initialization code) ...

    setupImGui(window);

    // ... (E57 loading code) ...

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // ... (ImGui rendering code) ...
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // UI window
        ImGui::Begin("Class Visibility");
        for (int i = 0; i < classCount; ++i)
        {
            ImGui::Checkbox(classNames[i].c_str(), &showClass[i]);
        }
        ImGui::End();

        // Clear screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Get window size
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // Setup camera/view
        setupCamera(width, height);

        // Render point cloud
        renderPointCloud();
        drawLine(0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
        //drawLine(0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f);
        //drawLine(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f);

        // Render UI
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // ... (OpenGL rendering code) ...

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}