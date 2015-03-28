#pragma once

namespace _462 {
	/**
	* Struct of the program options.
	*/
	struct Options
	{
		// whether to open a window or just render without one
		bool open_window;
		// not allocated, pointed it to something static
		const char* input_filename;
		// not allocated, pointed it to something static
		const char* output_filename;
		// window dimensions
		int width, height;
		int num_samples;

		//for depth of field
		bool dof_active;
		float dof_aperture_size;
		float dof_focal_length;		
		//-----

		//for glossy field 
		float glossy_surface_width;
		//------
	};

}