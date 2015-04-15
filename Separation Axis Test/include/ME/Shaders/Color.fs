#version 330

uniform vec3 Color;

out vec4 Fragment;

void main()
{
	Fragment = vec4(Color,1);

	return;
}