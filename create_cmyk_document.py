"""
Generate a two-page PDF for printing where each side is a solid CMYK color:
- Page 1: 100% cyan
- Page 2: 100% yellow

The script does not depend on external libraries; it writes the minimal PDF
structure directly so it can run in any standard Python environment.
"""

from pathlib import Path


def build_pdf_bytes() -> bytes:
    """Return the binary payload for the two-page CMYK PDF document."""
    width, height = 595.28, 841.89  # A4 in points (210 x 297 mm)
    media_box = f"[0 0 {width:.2f} {height:.2f}]"
    cyan_stream = (
        f"q\n0 0 {width:.2f} {height:.2f} re\n1 0 0 0 k\nf\nQ\n".encode()
    )
    yellow_stream = (
        f"q\n0 0 {width:.2f} {height:.2f} re\n0 0 1 0 k\nf\nQ\n".encode()
    )

    objects = []

    def obj(content: bytes) -> bytes:
        objects.append(content)
        return content

    obj(b"<< /Type /Catalog /Pages 2 0 R >>")
    obj("<< /Type /Pages /Count 2 /Kids [3 0 R 5 0 R] >>".encode())
    obj(
        f"<< /Type /Page /Parent 2 0 R /MediaBox {media_box} /Resources <<>> "
        f"/Contents 4 0 R >>".encode()
    )
    obj(
        b"<< /Length %d >>\nstream\n%sendstream\n"
        % (len(cyan_stream), cyan_stream)
    )
    obj(
        f"<< /Type /Page /Parent 2 0 R /MediaBox {media_box} /Resources <<>> "
        f"/Contents 6 0 R >>".encode()
    )
    obj(
        b"<< /Length %d >>\nstream\n%sendstream\n"
        % (len(yellow_stream), yellow_stream)
    )

    buffer = bytearray()
    buffer.extend(b"%PDF-1.4\n%\n")

    offsets = []
    for index, content in enumerate(objects, start=1):
        offsets.append(len(buffer))
        buffer.extend(f"{index} 0 obj\n".encode())
        buffer.extend(content)
        if not content.endswith(b"\n"):
            buffer.extend(b"\n")
        buffer.extend(b"endobj\n")

    xref_start = len(buffer)
    buffer.extend(b"xref\n")
    buffer.extend(f"0 {len(objects) + 1}\n".encode())
    buffer.extend(b"0000000000 65535 f \n")
    for offset in offsets:
        buffer.extend(f"{offset:010d} 00000 n \n".encode())

    buffer.extend(b"trailer\n")
    buffer.extend(f"<< /Size {len(objects) + 1} /Root 1 0 R >>\n".encode())
    buffer.extend(b"startxref\n")
    buffer.extend(f"{xref_start}\n".encode())
    buffer.extend(b"%%EOF\n")

    return bytes(buffer)


def create_cmyk_document(output_path: str | Path = "cmyk_printer_document.pdf") -> Path:
    """
    Create a PDF with one cyan side and one yellow side.

    Args:
        output_path: Destination file path. Defaults to ``cmyk_printer_document.pdf``.

    Returns:
        Path to the generated PDF.
    """
    output_path = Path(output_path)
    pdf_bytes = build_pdf_bytes()
    output_path.write_bytes(pdf_bytes)
    return output_path


if __name__ == "__main__":
    target = create_cmyk_document()
    print(f"Created CMYK PDF at {target.resolve()}")
