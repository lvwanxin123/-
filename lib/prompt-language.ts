import type {Prompt} from "@/types";
const chinesePattern=/[\u3400-\u9fff]/;
export function isChinesePrompt(prompt:Pick<Prompt,"title"|"content">){return chinesePattern.test(prompt.title)&&chinesePattern.test(prompt.content);}
