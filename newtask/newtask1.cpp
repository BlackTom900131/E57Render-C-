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

// For simplicity, error handling is minimal

// Define point structure
struct Point {
    double x, y, z;
    int label; // segmentation label
};

// Class definitions
const std::vector<std::string> classNames = {
    "Floor", "Ceiling", "Walls", "Furniture", "Beams", "Columns"
};
const int classCount = 6;

// Color map for classes
const float classColors[classCount][3] = {
    {0.58f, 0.29f, 0.0f},     // Floor - Brownish
    {1.0f, 1.0f, 0.0f},       // Ceiling - Yellow
    {1.0f, 0.0f, 0.0f},       // Walls - Red
    {0.0f, 1.0f, 0.0f},       // Furniture - Green
    {0.0f, 0.0f, 1.0f},       // Beams - Blue
    {1.0f, 0.65f, 0.0f}       // Columns - Orange
};

// Visibility toggles
bool showClass[classCount] = { true, true, true, true, true, true };

// Point cloud data
std::vector<Point> points;

// Function to load E57 file
//bool loadE57(const std::string& filename, std::vector<Point>& points) {
//    try {
//        //MessageBoxA(NULL, filename.c_str(), "Error", MB_OK | MB_ICONERROR);
//        // Create ReaderOptions object if needed
//        e57::ReaderOptions options;
//        e57::Reader reader(filename, options);
//        return true;
//    }
//    catch (const std::exception& e) {
//        std::cerr << "Error reading E57: " << e.what() << std::endl;
//        //MessageBoxA(NULL, e.what(), "Error", MB_OK | MB_ICONERROR);
//        return false;
//    }
//}

bool loadE57(const std::string& filename, std::vector<Point>& points) {
    try {
        e57::ReaderOptions options;
        e57::Reader reader(filename, options);

        const int64_t scanCount = reader.GetData3DCount();

        if (scanCount == 0) {
            std::cerr << "No 3D scans found in file.\n";
            return false;
        }

        // Read first scan
        e57::Data3D data3DHeader;

        reader.ReadData3D(0, data3DHeader);

        int64_t nColumn = 0;
        int64_t nRow = 0;

        int64_t nPointsSize = 0;        //Number of points


        int64_t nGroupsSize = 0;        //Number of groups
        int64_t nCountSize = 0;         //Number of points per group
        bool    bColumnIndex = false; //indicates that idElementName is "columnIndex"

        reader.GetData3DSizes(0, nRow, nColumn, nPointsSize, nGroupsSize, nCountSize, bColumnIndex);

        int64_t nSize = nRow;
        if (nSize == 0) nSize = 1024;    // choose a chunk size
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
        bool            bIntensity = false;
        double          intRange = 0;
        double          intOffset = 0;


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
        bool            bColor = false;
        int32_t         colorRedRange = 1;
        int32_t         colorRedOffset = 0;
        int32_t         colorGreenRange = 1;
        int32_t         colorGreenOffset = 0;
        int32_t         colorBlueRange = 1;
        int32_t         colorBlueOffset = 0;


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

        //struct Data3DPointsDouble {
        //    double* cartesianX = nullptr;
        //    double* cartesianY = nullptr;
        //    double* cartesianZ = nullptr;
        //    // Other optional fields
        //    double* intensity = nullptr;
        //    bool* isInvalid = nullptr;
        //    uint16_t* rowIndex = nullptr;
        //    uint16_t* columnIndex = nullptr;
        //    // etc.
        //};

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
                buffers
            );
            int64_t		count = 0;
            unsigned long size = 0;
            int			col = 0;
            int			row = 0;

            while ((size = dataReader.read()) > 0)	//Each call to dataReader.read() will retrieve the next column of data.
            {
                for (long i = 0; i < size; i++)
                {
                    Point p;
                    if (columnIndex)
                        col = columnIndex[i];
                    else
                        col = 0;	//point cloud case

                    if (rowIndex)
                        row = rowIndex[i];
                    else
                        row = count;	//point cloud case

                    if (isInvalidData != NULL)
                        //pScan->SetPoint(row, col, xData[i], yData[i], zData[i]);
                        p.x = xData[i];

                    if (bIntensity) {		//Normalize intensity to 0 - 1.
                        double intensity = (intData[i] - intOffset) / intRange;
                        //pScan->SetIntensity(row, col, intensity);
                    }

                    if (bColor) {			//Normalize color to 0 - 255
                        int red = ((redData[i] - colorRedOffset) * 255) / colorRedRange;
                        int green = ((greenData[i] - colorGreenOffset) * 255) / colorBlueRange;
                        int blue = ((blueData[i] - colorBlueOffset) * 255) / colorBlueRange;
                        //pScan->SetColor(row, col, red, green, blue);

                        count++;
                    }
                }
            }
        }

        

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error reading E57: " << e.what() << std::endl;
        return false;
    }
}
bool loadE571(const std::string& filename, std::vector<Point>& points) {
    e57::ReaderOptions options;
    e57::Reader reader("C_001.e57", options);
    //e57::Reader reader("C_001.e57");
    int scanIndex = 0;

    e57::Data3D scanHeader;
    reader.ReadData3D(scanIndex, scanHeader);
    int64_t nRow = scanHeader.pointCount;

    std::vector<double> xData(nRow);
    std::vector<double> yData(nRow);
    std::vector<double> zData(nRow);

    e57::Data3DPointsDouble buffers{};
    buffers.cartesianX = xData.data();
    buffers.cartesianY = yData.data();
    buffers.cartesianZ = zData.data();

    e57::CompressedVectorReader dataReader = reader.SetUpData3DPointsData(
        scanIndex,
        nRow,
        buffers
    );
    return true;
}


// Dummy segmentation function (simulate calling Python or ML model)
void segmentPoints(std::vector<Point>& pts) {
    // For demo, assign random classes
    for (auto& p : pts) {
        p.label = rand() % classCount;
    }
}

// Initialize GLFW and GLEW
GLFWwindow* initGL() {
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW\n";
        return nullptr;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Point Cloud Segmentation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return nullptr;
    }
    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to init GLEW\n";
        return nullptr;
    }
    return window;
}

// Setup ImGui
void setupImGui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

// Render point cloud
void renderPointCloud() {
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (const auto& p : points) {
        if (p.label >= 0 && showClass[p.label]) {

            glColor3f(
                classColors[p.label][0],
                classColors[p.label][1],
                classColors[p.label][2]
            );
            glVertex3f(p.x, p.y, p.z);
        }
    }
    glEnd();
}


// Function to get the first argument (excluding the program name)
std::string GetFirstArgument() {
    int argc = 0;
    LPWSTR* argvW = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (argvW == nullptr || argc < 2) {
        if (argvW) LocalFree(argvW);
        return "";
    }

    // Convert wide string to multi-byte string
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, argvW[1], -1, NULL, 0, NULL, NULL);
    std::string argument;
    if (size_needed > 0) {
        char* buffer = new char[size_needed];
        WideCharToMultiByte(CP_UTF8, 0, argvW[1], -1, buffer, size_needed, NULL, NULL);
        argument = buffer;
        delete[] buffer;
    }

    LocalFree(argvW);
    return argument;
}

// WinMain function (Win32 entry point)
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    std::string filename = "C_001.e57";
    //std::string filename = GetFirstArgument();
    //if (filename.empty()) {
    //    //MessageBoxA(NULL, "Usage: program.exe <e57_filename>", "Error", MB_OK | MB_ICONERROR);
    //    return -1;
    //}

    // Your program logic here
    // For example:
    //MessageBoxA(NULL, ("Loading file: " + filename).c_str(), "Info", MB_OK);

    // Load point cloud
    std::cout << "Loading E57 file...\n";
    if (!loadE57(filename, points)) {
        std::cerr << "Failed to load E57\n";
        //MessageBoxA(NULL, ("Failed to load E57 : " + filename).c_str(), "Info", MB_OK);
        return -1;
    }
    std::cout << "Loaded " << points.size() << " points.\n";

    // Perform segmentation (here, dummy)
    std::cout << "Segmenting points...\n";
    segmentPoints(points);
    std::cout << "Segmentation complete.\n";

    // Initialize GLFW (required for window creation)
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    // ... (other GLFW initialization, such as window creation) ...


    // ... (OpenGL initialization code) ...

    glfwSwapInterval(1);

    // Initialize OpenGL & ImGui
    GLFWwindow* window = initGL();
    if (!window) return -1;

    // ... (ImGui initialization code) ...
    
    setupImGui(window);

    // ... (E57 loading code) ...

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // ... (ImGui rendering code) ...
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // UI window
        ImGui::Begin("Class Visibility");
        for (int i = 0; i < classCount; ++i) {
            ImGui::Checkbox(classNames[i].c_str(), &showClass[i]);
        }
        ImGui::End();

        // Clear screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Setup camera/view
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.f, 0.f, -50.f);
        glRotatef(20.f, 1.f, 0.f, 0.f);
        glRotatef((float)glfwGetTime() * 10.f, 0.f, 1.f, 0.f);

        // Render point cloud
        renderPointCloud();

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