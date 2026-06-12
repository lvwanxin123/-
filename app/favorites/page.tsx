import Link from "next/link";
import {redirect} from "next/navigation";
import PromptCard from "@/components/PromptCard";
import {createClient} from "@/lib/supabase/server";
import type {Prompt} from "@/types";
export const dynamic="force-dynamic";
export default async function Favorites(){const supabase=createClient();const {data:{user}}=await supabase.auth.getUser();if(!user)redirect("/");const {data}=await supabase.from("user_favorites").select("prompts(*)").eq("user_id",user.id).order("created_at",{ascending:false});const prompts=((data??[]).map((row:any)=>row.prompts).filter(Boolean)) as Prompt[];return <main className="min-h-screen bg-zinc-50 px-4 py-8"><div className="mx-auto max-w-7xl"><Link href="/" className="text-sm text-[#6366f1]">← 返回首页</Link><h1 className="mt-6 text-3xl font-bold">我的收藏（{prompts.length} 条）</h1>{prompts.length?<div className="mt-8 grid gap-5 sm:grid-cols-2 lg:grid-cols-3">{prompts.map(p=><PromptCard key={p.id} prompt={p} isFavorited />)}</div>:<div className="mt-8 rounded-2xl border border-dashed bg-white p-10 text-center"><Link href="/" className="font-semibold text-[#6366f1]">还没有收藏，去首页发现好用的提示词 →</Link></div>}</div></main>}
