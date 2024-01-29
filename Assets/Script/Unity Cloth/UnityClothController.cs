using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnityClothController : MonoBehaviour
{

    private Mesh mesh;          // for front side
    //private Mesh reverseMesh;   // for back side
    private int nodeCount;
    private int[] triangles;         // for front side
    private int[] reverseTriangles;  // for back side
    private Vector3[] nodeArray;


    [Header("Cloth Parameters")]
    //node per row and col
    public int rows = 32;
    public int columns = 32;
    public Cloth cloth;
    public Material material;

    ClothSkinningCoefficient[] coefficients;

    private void Start()
    {
        //material = this.GetComponent<Material>();
        Generate();
    }
    private void Generate()
    {
        mesh = new Mesh();
        // reverseMesh = new Mesh();
        mesh.name = "Cloth";
        //reverseMesh.name = "backside_cloth";

        nodeCount = (columns + 1) * (rows + 1);
        nodeArray = new Vector3[nodeCount];
        Vector3[] normals = new Vector3[nodeCount];
        Vector2[] uv = new Vector2[nodeCount];
        Vector4[] tangents = new Vector4[nodeCount];
        float dx = 10f / (float)(columns - 1);
        float dy = 10f / (float)(rows - 1);
        Matrix4x4 Trans = Matrix4x4.Translate(new Vector3(-5, 0, -5));
        int index = 0;
        for (int i = 0, j = 0; j < rows + 1; j++)
        {
            for (int k = 0; k < columns + 1; k++, i++)
            {
                nodeArray[index] = new Vector3(j * dx, 0, k * dy);
                nodeArray[index] = Trans.MultiplyPoint3x4(nodeArray[i]);
                tangents[index] = new Vector4(1f, 0f, 0f, 1f);
                uv[i] = new Vector2((nodeArray[index].x - 10 / 2) / 10,
                                             (nodeArray[index].z - 10 / 2) / 10);
                normals[index] = new Vector3(0, 1, 0);

                index++;
            }
        }

        mesh.vertices = nodeArray;
        //reverseMesh.vertices = nodeArray;
        mesh.normals = normals;
        mesh.uv = uv;
        mesh.tangents = tangents;

        //reverseMesh.uv = uv;
        //reverseMesh.tangents = tangents;

        triangles = new int[2 * columns * rows * 6];
        //reverseTriangles = new int[2 * columns * rows * 6];

        index = 0;

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                triangles[index++] = i * (columns + 1) + j;
                triangles[index++] = (i + 1) * (columns + 1) + (j + 1);
                triangles[index++] = (i + 1) * (columns + 1) + j;

                triangles[index++] = i * (columns + 1) + j;
                triangles[index++] = i * (columns + 1) + (j + 1);
                triangles[index++] = (i + 1) * (columns + 1) + (j + 1);

                //triangles[index++] = i * (columns + 1) + j;
                //triangles[index++] = (i + 1) * (columns + 1) + j;
                //triangles[index++] = (i + 1) * (columns + 1) + (j + 1);

                //triangles[index++] = i * (columns + 1) + j;
                //triangles[index++] = (i + 1) * (columns + 1) + (j + 1);
                //triangles[index++] = i * (columns + 1) + (j + 1);
            }
        }
        //

        mesh.triangles = triangles;
        //mesh.RecalculateNormals();

        //reverseMesh.triangles = reverseTriangles;
        //reverseMesh.RecalculateNormals();

        GetComponent<SkinnedMeshRenderer>().material = material;
        GetComponent<SkinnedMeshRenderer>().sharedMesh = mesh;

    }


    // Update is called once per frame
    void Update()
    {
        // fixed node
        coefficients = cloth.coefficients;
        //coefficients[rows].maxDistance = 0f;
        //coefficients[((rows+1)* (columns+1)) -1].maxDistance = 0f;

        coefficients[rows * (columns + 1)].maxDistance = 0f;
        coefficients[0].maxDistance = 0f;

        cloth.coefficients = coefficients;

        GetComponent<SkinnedMeshRenderer>().sharedMesh = mesh;
    }

    private void OnEnable()
    {

    }
}
