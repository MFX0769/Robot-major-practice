import { tool } from "@opencode-ai/plugin";
import path from "path";
import { execSync } from "child_process";

export default tool({
  description: "用 Kimi 视觉模型读取图片内容。支持 jpg/png/gif/webp/bmp",
  args: {
    imagePath: tool.schema.string().describe("图片文件路径"),
    prompt: tool.schema.string().optional().describe("提问内容，默认：请描述这张图片"),
  },
  async execute(args, context) {
    const script = path.join(context.worktree, ".opencode/tools/kimi_vision.py");
    const prompt = args.prompt || "请详细描述这张图片里的内容";
    const result = execSync(`python "${script}" "${args.imagePath}" "${prompt}"`, {
      encoding: "utf-8",
      maxBuffer: 10 * 1024 * 1024,
    });
    return result.trim();
  },
});
