// Each lidar ray is drawn as a point that lands exactly on a pixel corner
// (and, for a 1-row scan, on the top edge of the target). Whether such a
// point covers a pixel depends on driver rounding; Mesa's D3D12 driver (WSL)
// drops nearly all of them. Shift by half a pixel onto pixel centres (+y:
// Ogre flips Y when rendering to a texture under GL).
uniform vec4 viewportSize; // (w, h, 1/w, 1/h)

varying float tex;

void main()
{
  gl_Position = ftransform();
  gl_Position.xy += vec2(viewportSize.z, viewportSize.w) * gl_Position.w;
  tex = gl_Vertex.x;
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
