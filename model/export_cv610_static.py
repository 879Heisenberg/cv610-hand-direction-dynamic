from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

import onnx

# Ultralytics chooses onnxruntime-gpu whenever CUDA is installed, even when
# export and validation explicitly run on CPU. The CPU runtime is sufficient
# for this static graph export, so avoid a blocking network auto-install.
os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")

from ultralytics import YOLO


CV610_SIZE = 416
UNSUPPORTED_CPU_OPS = {"TopK", "GatherElements", "Mod"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def tensor_description(value_info: onnx.ValueInfoProto) -> dict:
    tensor_type = value_info.type.tensor_type
    dimensions = []
    for dimension in tensor_type.shape.dim:
        dimensions.append(dimension.dim_value if dimension.HasField("dim_value") else dimension.dim_param)
    return {
        "name": value_info.name,
        "shape": dimensions,
        "elem_type": tensor_type.elem_type,
    }


def validate_export(path: Path) -> dict:
    model = onnx.load(str(path), load_external_data=True)
    onnx.checker.check_model(model)

    inputs = [tensor_description(item) for item in model.graph.input]
    outputs = [tensor_description(item) for item in model.graph.output]
    operators = sorted({node.op_type for node in model.graph.node})
    forbidden = sorted(UNSUPPORTED_CPU_OPS.intersection(operators))

    if len(inputs) != 1 or inputs[0]["shape"] != [1, 3, CV610_SIZE, CV610_SIZE]:
        raise RuntimeError(f"unexpected CV610 input contract: {inputs}")
    if len(outputs) != 1:
        raise RuntimeError(f"expected one raw detection output, got: {outputs}")
    if forbidden:
        raise RuntimeError(f"end-to-end/CPU-heavy operators still present: {forbidden}")

    return {
        "input": inputs[0],
        "output": outputs[0],
        "operators": operators,
        "forbidden_operators_found": forbidden,
    }


def export_static(weights: Path, destination: Path, force: bool) -> dict:
    if not weights.is_file():
        raise FileNotFoundError(weights)
    if destination.exists() and not force:
        raise FileExistsError(f"{destination} already exists; pass --force to replace it")

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="cv610-static-export-") as temp_name:
        temp_dir = Path(temp_name)
        staged_weights = temp_dir / weights.name
        shutil.copy2(weights, staged_weights)

        model = YOLO(str(staged_weights))
        if not getattr(model.model, "end2end", False):
            raise RuntimeError("source checkpoint is not an end-to-end YOLO26 model")

        exported = Path(
            model.export(
                format="onnx",
                imgsz=CV610_SIZE,
                batch=1,
                device="cpu",
                half=False,
                dynamic=False,
                simplify=True,
                opset=19,
                nms=False,
                end2end=False,
            )
        ).resolve()
        if not exported.is_file():
            raise RuntimeError(f"Ultralytics did not produce an ONNX file: {exported}")

        shutil.copy2(exported, destination)

    metadata = validate_export(destination)
    metadata.update(
        {
            "source_weights": str(weights.resolve()),
            "source_weights_sha256": sha256(weights),
            "artifact": str(destination.resolve()),
            "artifact_sha256": sha256(destination),
            "artifact_size_bytes": destination.stat().st_size,
            "deployment_size": [CV610_SIZE, CV610_SIZE],
            "export": {
                "opset": 19,
                "dynamic": False,
                "batch": 1,
                "nms": False,
                "end2end": False,
            },
        }
    )
    metadata_path = destination.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a CV610 static YOLO26 ONNX without NMS/TopK.")
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    metadata = export_static(args.weights, args.output.resolve(), args.force)
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
