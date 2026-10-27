// happy-dom lacks some APIs the code paths touch; keep stubs minimal.
Element.prototype.setPointerCapture = Element.prototype.setPointerCapture || function () {}
if (!globalThis.ResizeObserver) {
  globalThis.ResizeObserver = class { observe() {} unobserve() {} disconnect() {} }
}
