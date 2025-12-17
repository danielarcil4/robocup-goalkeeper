RoboCup Goalkeeper firmware
===========================

This folder contains the firmware source for the RoboCup Goalkeeper project.

Documentation
-------------

This repository uses Doxygen to generate API and code documentation for the entire project (excluding generated protobuf files and third-party components).

To generate the documentation locally:

- Linux/macOS (bash):

	```sh
	./scripts/generate_docs.sh
	```

- Windows (PowerShell):

	```powershell
	./scripts/generate_docs.ps1
	```

Generated HTML will be placed in `docs/html/index.html`.

Notes:
- The Doxygen configuration is located in `Doxyfile` at the repository root.
- `protobuf/*.pb.c`/`.pb.h` and `components/nanopb` are excluded by the Doxygen config.

