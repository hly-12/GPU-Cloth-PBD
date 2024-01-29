using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

public class GPUPBDCloth : MonoBehaviour
{
    // Start is called before the first frame update

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };

    [Header("Simulation Parameters")]
    public float timestep = 0.02f;
    public int iterationNum = 5;
    public float vertexMass = 1;

    [Header("External Forces")]
    public Vector3 gravity;

    [Header("Cloth Parameters")]
    public bool useExistingMesh;
    public int rows = 10;
    public int columns = 10;

    [Header("Distance Constraint")]
    public float distanceCompressionStiffness = 0.8f;
    public float distanceStretchStiffness = 0.8f;

    [Header("Bending Constraint")]
    public float bendingStiffness = 0.1f;
    public BendingMethod bendingMethod;

    [Header("Velocity Damping")]
    public DampingMethod dampingMethod;
    public float dampingStiffness = 0.02f;

    [Header("Point Constraints")]
    public PointConstraintType pointConstraintType;
    public int[] pointConstraintCustomIndices;
    public bool enableMouseInteraction;
    private int tempPointConstraint = -1;
    private Vector3 deltaPointConstraint, lastMousePos;

    [Header("Collision")]
    public GameObject[] collidableObjects;

     // TODO: figure out what the best value for this is

    // simulation data
    private float nextFrameTime = 0f;
    private Vector3[] positions;
    private Vector3[] velocities;
    private float[] frictions;
    private float invMass;
    private int numParticles;
    //
    private int numDistanceConstraints, numBendingConstraints;
    private int numCollidableSpheres, numCollidableCubes, numPointConstraints;

    [Header("Mesh Object")]
    public Material material;

    [Header("Compute Shader")]
    public ComputeShader computeShaderPBD;
    public int workGroupSize =8;

    //temp variable
    private Vector3[] deltaPositionArray;    // The array that stores all the deltas
    private int[] deltaCounterArray;              // The array to initialize delta count buffer
    private UInt3Struct[] deltaPosUintArray;      // The array to initialize deltaposInt buffer

    // constraints
    private DistanceConstraintStruct[] distanceConstraints;
    private BendingConstraintStruct[] bendingConstraints;
    private int[] pointConstraints;

    // compute buffers
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer projectedPositionsBuffer;
    private ComputeBuffer velocitiesBuffer;
    private ComputeBuffer deltaPositionsBuffer;
    private ComputeBuffer deltaPositionsUIntBuffer;
    private ComputeBuffer deltaCounterBuffer;
    private ComputeBuffer distanceConstraintsBuffer;
    private ComputeBuffer bendingConstraintsBuffer;
    private ComputeBuffer collidableSpheresBuffer;
    private ComputeBuffer collidableCubesBuffer;
    private ComputeBuffer pointConstraintsBuffer;
    private ComputeBuffer frictionsBuffer;
    private ComputeBuffer triangleBuffer;
    private ComputeBuffer triangleIndicesBuffer;



    // kernel IDs
    private int applyExternalForcesKernel;
    private int dampVelocitiesKernel;
    private int applyExplicitEulerKernel;
    private int projectConstraintDeltasKernel;
    private int averageConstraintDeltasKernel;
    private int satisfyPointConstraintsKernel;
    private int satisfySphereCollisionsKernel;
    private int satisfyCubeCollisionsKernel;
    private int updatePositionsKernel;
    private int updateNormalsKernel;

    // num of work groups
    private int numGroups_Vertices;
    private int numGroups_DistanceConstraints;
    private int numGroups_BendingConstraints;
    private int numGroups_AllConstraints;
    private int numGroups_PointConstraints;

    // mesh data
    private Mesh mesh;
    private Mesh reverseMesh;
    private Triangle[] triangles;

    private ComputeShader PBDClothSolver;
    private int frame = 0;


    List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
    List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
   

    //new properties

    int[] triArray;
    vertData[] vDataArray;

    ComputeBuffer vertsBuff = null;
    ComputeBuffer triBuffer = null;

    void Start()
    {
        // create new mesh
        //material = new Material(renderShader); // new material for difference object
        //material.SetTexture(0, texture); //set color to material
        PBDClothSolver = Instantiate(computeShaderPBD); // to instantiate the compute shader to be use with multiple object

        //init nodedata
        initClothData();

        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);

        invMass = 1.0f / vertexMass;

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);

        initDistanceConstraint();
        initBendingConstraint();

        //write to file
        //write distance constraint


        //write bending constraint
        //

        SetupComputeBuffers();
        print("numDConstraints = " + numDistanceConstraints);
        print("numBConstraints = " + numBendingConstraints);

    }
    void initClothData()
    {
        float dx = 10f / (float)(columns - 1);
        float dy = 10f / (float)(rows - 1);
        int vertexCount = (rows) * (columns);
        int triCount = (rows - 1) * (columns - 1) * 6;
        Vector3[] vertices = new Vector3[vertexCount];
        Vector3[] normals = new Vector3[vertexCount];
        int[] triangleArr = new int[triCount];
        Matrix4x4 Trans = Matrix4x4.Translate(new Vector3(-5, 0, -5));
        Vector2[] texcor = new Vector2[vertexCount];
        vDataArray = new vertData[vertexCount];
        triArray = new int[triCount];
        
        //position
        int index = 0;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {

                Vector3 vertex = new Vector3(j * dx, 0, i * dy);
                vertices[index] = Trans.MultiplyPoint3x4(vertex);
                //texcor[index] = new Vector2((vertices[index].x - 10.0f / 2.0f) / 10.0f,
                //                             -(vertices[index].z - 10.0f / 2.0f) / 10.0f);

                //Vector3 vertex = new Vector3(j * dx, 0, i * dy);
                //vertices[index] = Trans.MultiplyPoint3x4(vertex);
                texcor[index] = new Vector2((vertices[index].x - 10 / 2) / 10,
                                             (vertices[index].z - 10 / 2) / 10);

                texcor[index] *= -1; // flipping texture
                normals[index] = Vector3.one;
                //new
                vDataArray[index] = new vertData();
                vDataArray[index].pos = vertices[index];
                vDataArray[index].norms = Vector3.zero;
                vDataArray[index].uvs = texcor[index];
                index++;
            }
        }



        index = 0;
        List<Triangle> tmpTriangles = new List<Triangle>();
        for (int i = 0; i < rows - 1; i++)
        {
            for (int j = 0; j < columns - 1; j++)
            {
               triArray[index++] = i * (columns) + j;
               triArray[index++] = (i + 1) * (columns) + j;
               triArray[index++] = (i + 1) * (columns) + (j + 1);
               triArray[index++] = i * (columns) + j;
               triArray[index++] = (i + 1) * (columns) + (j + 1);
                triArray[index++] = i * (columns) + (j + 1);

                tmpTriangles.Add(new Triangle(
                    i * (columns) + j, 
                    (i + 1) * (columns) + j,
                    (i + 1) * (columns) + (j + 1)));
                tmpTriangles.Add(new Triangle(
                   i * (columns) + j,
                   (i + 1) * (columns) + (j + 1),
                   i * (columns) + (j + 1)));
            }
        }
        numParticles = vertexCount;

        positions = new Vector3[numParticles];
        velocities = new Vector3[numParticles];
        frictions = new float[numParticles];
        deltaPositionArray = new Vector3[numParticles];
        deltaPosUintArray = new UInt3Struct[numParticles];
        deltaCounterArray = new int[numParticles];

        positions = vertices;
        triangles = tmpTriangles.ToArray();

        initTrianglePtr.Add(0);
        for (int i = 0; i < numParticles; i++)
        {
            foreach (Triangle tri in triangles)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                {
                    MTriangle tmpTri = new MTriangle();
                    tmpTri.v0 = tri.vertices[0];
                    tmpTri.v1= tri.vertices[1];
                    tmpTri.v2 = tri.vertices[2];
             
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }


        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);

        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);

        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);

    }
    void initDistanceConstraint()
    {
        //distance
        // use a set to get unique edges
        HashSet<Edge> edgeSet = new HashSet<Edge>(new EdgeComparer());
        for (int i = 0; i < triangles.Length; i++)
        {
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[1]));
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[2]));
            edgeSet.Add(new Edge(triangles[i].vertices[1], triangles[i].vertices[2]));
        };

        numDistanceConstraints = edgeSet.Count;
        distanceConstraints = new DistanceConstraintStruct[edgeSet.Count];
        int j = 0;
        foreach (Edge e in edgeSet)
        {
            EdgeStruct edge;
            edge.startIndex = e.startIndex;
            edge.endIndex = e.endIndex;
            distanceConstraints[j].edge = edge;
            distanceConstraints[j].restLength = (positions[edge.startIndex] - positions[edge.endIndex]).magnitude;
            j++;
        }
    }
    void initBendingConstraint()
    {
        Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

        // map edges to all of the faces to which they are connected
        foreach (Triangle tri in triangles)
        {
            Edge e1 = new Edge(tri.vertices[0], tri.vertices[1]);
            if (wingEdges.ContainsKey(e1) && !wingEdges[e1].Contains(tri))
            {
                wingEdges[e1].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e1, tris);
            }

            Edge e2 = new Edge(tri.vertices[0], tri.vertices[2]);
            if (wingEdges.ContainsKey(e2) && !wingEdges[e2].Contains(tri))
            {
                wingEdges[e2].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e2, tris);
            }

            Edge e3 = new Edge(tri.vertices[1], tri.vertices[2]);
            if (wingEdges.ContainsKey(e3) && !wingEdges[e3].Contains(tri))
            {
                wingEdges[e3].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e3, tris);
            }
        }

        // wingEdges are edges with 2 occurences,
        // so we need to remove the lower frequency ones
        List<Edge> keyList = wingEdges.Keys.ToList();
        foreach (Edge e in keyList)
        {
            if (wingEdges[e].Count < 2)
            {
                wingEdges.Remove(e);
            }
        }

        numBendingConstraints = wingEdges.Count;
        bendingConstraints = new BendingConstraintStruct[numBendingConstraints];
        int j = 0;
        foreach (Edge wingEdge in wingEdges.Keys)
        {
            /* wingEdges are indexed like in the Bridson,
             * Simulation of Clothing with Folds and Wrinkles paper
             *    3
             *    ^
             * 0  |  1
             *    2
             */

            int[] indices = new int[4];
            indices[2] = wingEdge.startIndex;
            indices[3] = wingEdge.endIndex;

            int b = 0;
            foreach (Triangle tri in wingEdges[wingEdge])
            {
                for (int i = 0; i < 3; i++)
                {
                    int point = tri.vertices[i];
                    if (point != indices[2] && point != indices[3])
                    {
                        //tri #1
                        if (b == 0)
                        {
                            indices[0] = point;
                            break;
                        }
                        //tri #2
                        else if (b == 1)
                        {
                            indices[1] = point;
                            break;
                        }
                    }
                }
                b++;
            }

            bendingConstraints[j].index0 = indices[0];
            bendingConstraints[j].index1 = indices[1];
            bendingConstraints[j].index2 = indices[2];
            bendingConstraints[j].index3 = indices[3];
            Vector3 p0 = positions[indices[0]];
            Vector3 p1 = positions[indices[1]];
            Vector3 p2 = positions[indices[2]];
            Vector3 p3 = positions[indices[3]];

            Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            float d = Vector3.Dot(n1, n2);
            d = Mathf.Clamp(d, -1.0f, 1.0f);
            bendingConstraints[j].restAngle = Mathf.Acos(d);

            j++;
        }
    }

    

    void SetupComputeBuffers()
    {

        // create the compute buffers
        positionsBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        projectedPositionsBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        velocitiesBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        frictionsBuffer = new ComputeBuffer(numParticles, sizeof(float));
        deltaPositionsBuffer = new ComputeBuffer(numParticles, sizeof(float) * 3);
        deltaPositionsUIntBuffer = new ComputeBuffer(numParticles, sizeof(uint) * 3);
        deltaCounterBuffer = new ComputeBuffer(numParticles, sizeof(int));
        distanceConstraintsBuffer = new ComputeBuffer(numDistanceConstraints, sizeof(float) + sizeof(int) * 2);
        bendingConstraintsBuffer = new ComputeBuffer(numBendingConstraints, sizeof(float) + sizeof(int) * 4);
        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleIndicesBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
       

        // fill buffers with initial data
        positionsBuffer.SetData(positions);
        projectedPositionsBuffer.SetData(positions);
        velocitiesBuffer.SetData(velocities);
        frictionsBuffer.SetData(frictions);
        deltaPositionsBuffer.SetData(deltaPositionArray);
        deltaPositionsUIntBuffer.SetData(deltaPosUintArray);
        deltaCounterBuffer.SetData(deltaCounterArray);
        distanceConstraintsBuffer.SetData(distanceConstraints);
        bendingConstraintsBuffer.SetData(bendingConstraints);

        triangleBuffer.SetData(initTriangle.ToArray());
        triangleIndicesBuffer.SetData(initTrianglePtr.ToArray());

        // identify the kernels
        applyExternalForcesKernel = PBDClothSolver.FindKernel("ApplyExternalForces");
        dampVelocitiesKernel = PBDClothSolver.FindKernel("DampVelocities");
        applyExplicitEulerKernel = PBDClothSolver.FindKernel("ApplyExplicitEuler");
        projectConstraintDeltasKernel = PBDClothSolver.FindKernel("ProjectConstraintDeltas");
        averageConstraintDeltasKernel = PBDClothSolver.FindKernel("AverageConstraintDeltas");
        updatePositionsKernel = PBDClothSolver.FindKernel("UpdatePositions");
        satisfySphereCollisionsKernel = PBDClothSolver.FindKernel("SatisfySphereCollisions");
        satisfyCubeCollisionsKernel = PBDClothSolver.FindKernel("SatisfyCubeCollisions");
        satisfyPointConstraintsKernel = PBDClothSolver.FindKernel("SatisfyPointConstraints");
        updateNormalsKernel = PBDClothSolver.FindKernel("updateNormals");   //just add

        // set uniform data for kernels
        PBDClothSolver.SetInt("numParticles", numParticles);
        PBDClothSolver.SetInt("numDistanceConstraints", numDistanceConstraints);
        PBDClothSolver.SetInt("numBendingConstraints", numBendingConstraints);
        PBDClothSolver.SetInt("numAllConstraints", numDistanceConstraints + numBendingConstraints);


        // bind buffer data to each kernel
        PBDClothSolver.SetBuffer(applyExternalForcesKernel, "velocities", velocitiesBuffer);

        PBDClothSolver.SetBuffer(dampVelocitiesKernel, "velocities", velocitiesBuffer);

        PBDClothSolver.SetBuffer(applyExplicitEulerKernel, "positions", positionsBuffer);
        PBDClothSolver.SetBuffer(applyExplicitEulerKernel, "projectedPositions", projectedPositionsBuffer);
        PBDClothSolver.SetBuffer(applyExplicitEulerKernel, "velocities", velocitiesBuffer);

        PBDClothSolver.SetBuffer(projectConstraintDeltasKernel, "projectedPositions", projectedPositionsBuffer);
        PBDClothSolver.SetBuffer(projectConstraintDeltasKernel, "deltaPos", deltaPositionsBuffer);
        PBDClothSolver.SetBuffer(projectConstraintDeltasKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);
        PBDClothSolver.SetBuffer(projectConstraintDeltasKernel, "deltaCount", deltaCounterBuffer);
        PBDClothSolver.SetBuffer(projectConstraintDeltasKernel, "distanceConstraints", distanceConstraintsBuffer);
        PBDClothSolver.SetBuffer(projectConstraintDeltasKernel, "bendingConstraints", bendingConstraintsBuffer);

        PBDClothSolver.SetBuffer(averageConstraintDeltasKernel, "projectedPositions", projectedPositionsBuffer);
        PBDClothSolver.SetBuffer(averageConstraintDeltasKernel, "deltaPos", deltaPositionsBuffer);
        PBDClothSolver.SetBuffer(averageConstraintDeltasKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);
        PBDClothSolver.SetBuffer(averageConstraintDeltasKernel, "deltaCount", deltaCounterBuffer);

        PBDClothSolver.SetBuffer(updatePositionsKernel, "positions", positionsBuffer);
        PBDClothSolver.SetBuffer(updatePositionsKernel, "projectedPositions", projectedPositionsBuffer);
        PBDClothSolver.SetBuffer(updatePositionsKernel, "velocities", velocitiesBuffer);
        PBDClothSolver.SetBuffer(updatePositionsKernel, "frictions", frictionsBuffer);

        PBDClothSolver.SetBuffer(updateNormalsKernel, "Triangles", triangleBuffer);
        PBDClothSolver.SetBuffer(updateNormalsKernel, "TrianglePtr", triangleIndicesBuffer);
        PBDClothSolver.SetBuffer(updateNormalsKernel, "positions", positionsBuffer);
        PBDClothSolver.SetBuffer(updateNormalsKernel, "vertsBuff", vertsBuff);

        

        //calculate and set the work group size
        numGroups_Vertices = Mathf.CeilToInt((float)numParticles / workGroupSize);
        numGroups_DistanceConstraints = Mathf.CeilToInt((float)numDistanceConstraints / workGroupSize);
        numGroups_BendingConstraints = Mathf.CeilToInt((float)numBendingConstraints / workGroupSize);
        numGroups_AllConstraints = Mathf.CeilToInt((float)(numDistanceConstraints + numBendingConstraints) / workGroupSize);
    }
    // Update is called once per frame
    void Update()
    {
        //if(frame < 1)
         

        PBDClothSolver.SetVector("gravity", gravity);
        PBDClothSolver.SetFloat("invMass", invMass);
        PBDClothSolver.SetFloat("stretchStiffness", distanceStretchStiffness);
        PBDClothSolver.SetFloat("compressionStiffness", distanceCompressionStiffness);
        PBDClothSolver.SetFloat("bendingStiffness", bendingStiffness);
        PBDClothSolver.SetFloat("dampingStiffness", dampingStiffness);

        // send the dt data to the GPU
        PBDClothSolver.SetFloat("dt", timestep);

        // step 5: apply external forces
        PBDClothSolver.Dispatch(applyExternalForcesKernel, numGroups_Vertices, 1, 1);
        // step 6: damp velocity
        if (dampingMethod != DampingMethod.noDamping)
        {
            PBDClothSolver.Dispatch(dampVelocitiesKernel, numGroups_Vertices, 1, 1);
        }
    

        //else if(dampingMethod =)
        // step 7: apply explicit Euler to positions based on velocity
        PBDClothSolver.Dispatch(applyExplicitEulerKernel, numGroups_Vertices, 1, 1);
        // step 8: clear current collisions and generate new collisions
        SetupCollisionComputeBuffers();

        // step 9-11: project constraints iterationNum times
        for (int j = 0; j < iterationNum; j++)
        {
            // TODO: maybe shuffle here

            // distance constraints
            PBDClothSolver.Dispatch(projectConstraintDeltasKernel, numGroups_AllConstraints, 1, 1);
            PBDClothSolver.Dispatch(averageConstraintDeltasKernel, numGroups_Vertices, 1, 1);

            // collision constraints
            if (numCollidableSpheres > 0)
            {
                PBDClothSolver.Dispatch(satisfySphereCollisionsKernel, numGroups_Vertices, 1, 1);
            }
            if (numCollidableCubes > 0)
            {
                PBDClothSolver.Dispatch(satisfyCubeCollisionsKernel, numGroups_Vertices, 1, 1);
            }
        }

        //satisfy pointConstraints
        if (numPointConstraints > 0)
        {
            if (enableMouseInteraction && tempPointConstraint != -1)
            {
                PBDClothSolver.SetBool("hasTempPointConstraint", true);
                PBDClothSolver.SetVector("deltaPointConstraint", deltaPointConstraint);
                PBDClothSolver.SetInt("tempPointConstraint", tempPointConstraint);
            }
            else
            {
                PBDClothSolver.SetBool("hasTempPointConstraint", false);
                PBDClothSolver.SetInt("tempPointConstraint", -1);
            }
            PBDClothSolver.Dispatch(satisfyPointConstraintsKernel, numGroups_PointConstraints, 1, 1);
        }
        // step 13 & 14: apply projected positions to actual vertices
        PBDClothSolver.Dispatch(updatePositionsKernel, numGroups_Vertices, 1, 1);


        // step 16: update all velocities using friction
        //ApplyFriction();

        //TODO: handle mouse drag inputs, since we dont have a mesh collider anymore we need to do this manually in the compute shader

        //Re-calculate normal
        PBDClothSolver.Dispatch(updateNormalsKernel, numGroups_Vertices, 1, 1);

        renderObject();
    }


    void renderObject()
    {
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);

    }

    private void AddPointConstraints()
    {
        List<int> points = new List<int>();

        if (pointConstraintType == PointConstraintType.topCorners)
        {
            points.Add(0);
            points.Add(columns-1);
        }
        else if (pointConstraintType == PointConstraintType.botomCorners)
        {
            points.Add((rows *columns) - 1);
            points.Add(((rows * columns) - 1) - (columns - 1));
        }
        else if (pointConstraintType == PointConstraintType.topRow)
        {
            for (int i = 0; i < columns; i++)
            {
                points.Add(i);
            }
        }
        else if (pointConstraintType == PointConstraintType.leftCorners)
        {
            //points.Add(0);
            //points.Add(rows * (columns + 1));
        }
        else if (pointConstraintType == PointConstraintType.leftRow)
        {
            //for (int i = 0; i < columns; i++)
            //{
            //    points.Add(i * (columns + 1)); 
            //}
        }

        for (int i = 0; i < pointConstraintCustomIndices.Length; i++)
        {
            int index = pointConstraintCustomIndices[i];
            if (index >= 0 && index < numParticles)
            {
                points.Add(index);
            }
        }

        if (tempPointConstraint != -1)
        {
            points.Add(tempPointConstraint);
        }

        numPointConstraints = points.Count;
        pointConstraints = new int[numPointConstraints];
        for (int i = 0; i < numPointConstraints; i++)
        {
            pointConstraints[i] = points[i];
        }
    }

    private void SetupCollisionComputeBuffers()
    {
        List<GameObject> spheres = new List<GameObject>();
        List<GameObject> cubes = new List<GameObject>();

        // categorize all the collidable objects
        for (int j = 0; j < collidableObjects.Length; j++)
        {
            if (!collidableObjects[j].activeSelf) continue;

            Collider collider = collidableObjects[j].GetComponent<Collider>();

            if (collider.GetType() == typeof(SphereCollider))
            {
                spheres.Add(collidableObjects[j]);
            }
            else if (collider.GetType() == typeof(BoxCollider))
            {
                cubes.Add(collidableObjects[j]);
            }
        }

        // create the compute buffer for spheres
        CollidableSphereStruct[] collidableSpheres = new CollidableSphereStruct[spheres.Count];
        numCollidableSpheres = spheres.Count;
        for (int i = 0; i < numCollidableSpheres; i++)
        {
            Vector3 center = spheres[i].transform.position + spheres[i].GetComponent<SphereCollider>().center;
            float radius = spheres[i].transform.lossyScale.x * spheres[i].GetComponent<SphereCollider>().radius;
            center = transform.InverseTransformPoint(center);
            
           
            collidableSpheres[i].center = center;
            collidableSpheres[i].radius = radius;
        }
        if (collidableSpheresBuffer != null) collidableSpheresBuffer.Release();

        if (numCollidableSpheres > 0)
        {
            collidableSpheresBuffer = new ComputeBuffer(numCollidableSpheres, sizeof(float) * 4);
            // fill buffers with initial data
            collidableSpheresBuffer.SetData(collidableSpheres);
            PBDClothSolver.SetInt("numCollidableSpheres", numCollidableSpheres);
            PBDClothSolver.SetBuffer(satisfySphereCollisionsKernel, "projectedPositions", projectedPositionsBuffer);
            PBDClothSolver.SetBuffer(satisfySphereCollisionsKernel, "collidableSpheres", collidableSpheresBuffer);
            PBDClothSolver.SetBuffer(satisfySphereCollisionsKernel, "frictions", frictionsBuffer);

        }

        // create the compute buffer for cubes
        CollidableCubeStruct[] collidableCubes = new CollidableCubeStruct[cubes.Count];
        numCollidableCubes = cubes.Count;
        for (int i = 0; i < numCollidableCubes; i++)
        {
            Vector3 cubeCenter = cubes[i].transform.position + cubes[i].GetComponent<BoxCollider>().center;
            cubeCenter = transform.InverseTransformPoint(cubeCenter);
            collidableCubes[i].center = cubeCenter;


            float extent_x = cubes[i].transform.lossyScale.x * cubes[i].GetComponent<BoxCollider>().size.x / 2f;
            float extent_y = cubes[i].transform.lossyScale.y * cubes[i].GetComponent<BoxCollider>().size.y / 2f;
            float extent_z = cubes[i].transform.lossyScale.z * cubes[i].GetComponent<BoxCollider>().size.z / 2f;
            collidableCubes[i].extent = new Vector3(extent_x, extent_y, extent_z);
        }
        if (collidableCubesBuffer != null) collidableCubesBuffer.Release();

        if (numCollidableCubes > 0)
        {
            collidableCubesBuffer = new ComputeBuffer(numCollidableCubes, sizeof(float) * 6);
            // fill buffers with initial data
            collidableCubesBuffer.SetData(collidableCubes);
            PBDClothSolver.SetInt("numCollidableCubes", numCollidableCubes);
            PBDClothSolver.SetBuffer(satisfyCubeCollisionsKernel, "projectedPositions", projectedPositionsBuffer);
            PBDClothSolver.SetBuffer(satisfyCubeCollisionsKernel, "collidableCubes", collidableCubesBuffer);
            PBDClothSolver.SetBuffer(satisfyCubeCollisionsKernel, "frictions", frictionsBuffer);
        }

        // create compute buffer for point constraints
        AddPointConstraints();
        if (pointConstraintsBuffer != null) pointConstraintsBuffer.Release();

        if (numPointConstraints > 0)
        {
            pointConstraintsBuffer = new ComputeBuffer(numPointConstraints, sizeof(int));
            pointConstraintsBuffer.SetData(pointConstraints);
            PBDClothSolver.SetInt("numPointConstraints", numPointConstraints);
            PBDClothSolver.SetBuffer(satisfyPointConstraintsKernel, "positions", positionsBuffer);
            PBDClothSolver.SetBuffer(satisfyPointConstraintsKernel, "projectedPositions", projectedPositionsBuffer);
            PBDClothSolver.SetBuffer(satisfyPointConstraintsKernel, "pointConstraints", pointConstraintsBuffer);
            numGroups_PointConstraints = Mathf.CeilToInt((float)numPointConstraints / workGroupSize);
        }

    }
    private void OnDestroy()
    {
        if (this.enabled)
        {
            positionsBuffer.Dispose();
            projectedPositionsBuffer.Dispose();
            velocitiesBuffer.Dispose();
            deltaPositionsBuffer.Dispose();
            deltaPositionsUIntBuffer.Dispose();
            deltaCounterBuffer.Dispose();
            distanceConstraintsBuffer.Dispose();
            bendingConstraintsBuffer.Dispose();
            if (collidableSpheresBuffer != null)
                collidableSpheresBuffer.Dispose();
            if (collidableCubesBuffer != null)
                collidableCubesBuffer.Dispose();
            if(pointConstraintsBuffer !=null)
                pointConstraintsBuffer.Dispose();

            frictionsBuffer.Dispose();
            triangleBuffer.Dispose();
            triangleIndicesBuffer.Dispose();

       
            vertsBuff.Dispose();
            triBuffer.Dispose();
        }
    }
}
